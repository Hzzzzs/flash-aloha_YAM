"""Exercise the evaluator with a fake robot; never opens a CAN interface."""

import contextlib
import csv
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np


SOURCE = Path(__file__).resolve().parents[1] / "10_gravity_comp_static_eval.py"
spec = importlib.util.spec_from_file_location("gravity_eval", SOURCE)
evaluation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluation)


class FakeClock:
    value = 0.0

    def monotonic(self):
        return self.value

    def sleep(self, duration):
        self.value += duration


class FakeKinematics:
    def fk(self, *args):
        return np.eye(4)


class FakeRobot:
    def __init__(self, feedback=2.3):
        self.reference = np.array([0.1, 0.3, 0.6, 0.2, 0.1, 0.0, 0.5])
        self.gravity_comp_factor = np.array([1, 1.2, 1.25, 1.1, 1.2, 1, 0])
        self._grav_comp_kd = np.array([0.1, 0.1, 0.1, 0.3, 0.05, 0.05, 0])
        self.closed = False
        self.commands = []
        self.idle_count = 0
        self.feedback = feedback
        self._command_lock = threading.Lock()
        self._commands = SimpleNamespace(torques=np.zeros(7), kp=np.zeros(7))
        state = SimpleNamespace(error_code="0x1", temp_mos=25, temp_rotor=25)
        self.motor_chain = SimpleNamespace(read_states=lambda: [state] * 7)

    def get_joint_pos(self):
        return self.reference.copy()

    def get_observations(self):
        q = self.reference.copy()
        q[2] -= 0.01
        return {"joint_pos": q, "joint_vel": np.zeros(7),
                "joint_eff": np.array([0, 1, self.feedback, 0, 0, 0, 0])}

    def get_motor_torques(self):
        return np.array([0, 1, 1.5, 0, 0, 0, 0]) + self._commands.torques

    def command_joint_state(self, command):
        self.commands.append(command)

    def enter_gravity_comp_idle(self):
        self.idle_count += 1

    def close(self):
        self.closed = True


class HoldTests(unittest.TestCase):
    def run_evaluation(self, directory, robot, **kwargs):
        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(evaluation, "run_startup_checks"))
            stack.enter_context(patch.object(evaluation, "combine_arm_and_gripper_xml", return_value="fake.xml"))
            stack.enter_context(patch.object(evaluation, "Kinematics", return_value=FakeKinematics()))
            stack.enter_context(patch.object(evaluation, "get_yam_robot", return_value=robot))
            stack.enter_context(patch.object(evaluation, "time", FakeClock()))
            stack.enter_context(patch("builtins.input", return_value=""))
            stack.enter_context(patch.object(evaluation.sys, "stdin", io.StringIO("\n\n")))
            stack.enter_context(patch.object(evaluation.select, "select", return_value=([True], [], [])))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            evaluation.main("can_right", directory, duration_s=5, **kwargs)

    def test_hold_uses_measured_pose_and_never_counts_as_gravity_pass(self):
        robot = FakeRobot()
        with tempfile.TemporaryDirectory() as directory:
            self.run_evaluation(directory, robot, poses=2, diagnostic_hold=True)
            metrics = json.loads((Path(directory) / "metrics.json").read_text())
            with (Path(directory) / "samples.csv").open() as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(robot.commands), 2)
            for command in robot.commands:
                np.testing.assert_array_equal(command["pos"], robot.reference)
                self.assertEqual(command["kp"][6], 0)
                self.assertEqual(command["kd"][6], 0)
            self.assertEqual(robot.idle_count, 2)
            self.assertTrue(robot.closed)
            self.assertTrue(all(m["initial_acceptance"] is None for m in metrics))
            self.assertTrue(all(m["diagnostic_hold"] for m in metrics))
            self.assertAlmostEqual(float(rows[0]["pd3_nm"]), 0.8)
            self.assertAlmostEqual(float(rows[0]["estimated_total3_nm"]), 2.3)
            self.assertGreaterEqual(float(rows[-1]["t_s"]), 5)

    def test_feedback_torque_guard_closes_even_if_feedforward_is_small(self):
        robot = FakeRobot(feedback=21)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, "torque guard"):
                self.run_evaluation(directory, robot, poses=1, diagnostic_hold=True)
        self.assertTrue(robot.closed)

    def test_default_mode_does_not_enable_position_hold(self):
        robot = FakeRobot()
        with tempfile.TemporaryDirectory() as directory:
            self.run_evaluation(directory, robot, poses=1)
            metrics = json.loads((Path(directory) / "metrics.json").read_text())
        self.assertEqual(robot.commands, [])
        self.assertFalse(metrics[0]["diagnostic_hold"])
        self.assertTrue(metrics[0]["initial_acceptance"])
        self.assertTrue(robot.closed)

    def test_local_bias_changes_only_j3_feedforward_without_position_hold(self):
        robot = FakeRobot()
        with tempfile.TemporaryDirectory() as directory:
            self.run_evaluation(directory, robot, poses=1, j3_bias_nm=0.5)
            with (Path(directory) / "samples.csv").open() as handle:
                rows = list(csv.DictReader(handle))
            metrics = json.loads((Path(directory) / "metrics.json").read_text())
        self.assertEqual(robot.commands, [])
        np.testing.assert_array_equal(robot._commands.kp, np.zeros(7))
        np.testing.assert_array_equal(robot._commands.torques, [0, 0, 0.5, 0, 0, 0, 0])
        self.assertAlmostEqual(float(rows[-1]["estimated_total3_nm"]), 2.0)
        self.assertEqual(metrics[0]["j3_bias_nm"], 0.5)
        self.assertTrue(robot.closed)

    def test_invalid_bias_options_fail_before_hardware_construction(self):
        for kwargs in [dict(j3_bias_nm=float("nan")), dict(j3_bias_nm=0.81),
                       dict(j3_bias_nm=0.5, diagnostic_hold=True), dict(j3_bias_nm=0.5, poses=2)]:
            with self.subTest(kwargs=kwargs), patch.object(evaluation, "get_yam_robot") as getter:
                with self.assertRaises(SystemExit):
                    evaluation.main("can_right", "/unused", **kwargs)
                getter.assert_not_called()


if __name__ == "__main__":
    unittest.main()
