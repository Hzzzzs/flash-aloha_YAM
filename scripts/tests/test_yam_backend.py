"""Camera-free YAM backend tests; no CAN interface or motor is opened."""
import threading
import time
import unittest
from types import SimpleNamespace

import numpy as np

from hardware.arm_factory import build_arms, normalize_hardware_config
from hardware.backends.yam import YAMBackend, YAMController


class FakeRobot:
    def __init__(self):
        self._kp = np.array([80, 80, 80, 40, 10, 10, 20.0])
        self._kd = np.array([5, 5, 5, 3, 1.5, 1.5, 0.5])
        self._grav_comp_kd = np.array([.1, .1, .1, .3, .05, .05, 0.0])
        self._state_lock = threading.Lock()
        self._joint_state = SimpleNamespace(
            pos=np.r_[np.zeros(6), 1.0], vel=np.zeros(7), timestamp=time.time())
        self.commands = []
        self.passive = 0
        self.closed = 0

    def command_joint_state(self, command):
        copied = {key: np.asarray(value, dtype=float).copy() for key, value in command.items()}
        self.commands.append(copied)
        with self._state_lock:
            self._joint_state = SimpleNamespace(
                pos=copied['pos'].copy(), vel=copied['vel'].copy(), timestamp=time.time())

    def get_joint_state(self):
        return {'pos': self._joint_state.pos.copy(), 'vel': self._joint_state.vel.copy()}

    def enter_gravity_comp_idle(self):
        self.passive += 1

    def close(self):
        self.closed += 1


class FakeController:
    def __init__(self, role):
        self.role = role
        self.q = np.r_[np.zeros(6), 1.0]
        self.commands = []
        self.gripper_commands = []
        self.passive = self.safe = self.holds = self.homes = self.closed = 0

    def read_state(self):
        return self.q.copy(), np.zeros(7), time.time()

    def command_target(self, target, *, kp_scale=1.0, kd_scale=1.0):
        self.q = np.asarray(target, dtype=float).copy()
        self.commands.append((self.q.copy(), kp_scale, kd_scale))

    def hold_current(self):
        self.holds += 1
        return self.q.copy()

    def enter_passive(self):
        self.passive += 1

    def command_passive_gripper(self, opening):
        self.gripper_commands.append(opening)

    def enter_safe_stop(self):
        self.safe += 1

    def check(self):
        pass

    def go_home(self):
        self.homes += 1

    def close(self):
        self.closed += 1


def context(double=True):
    return SimpleNamespace(
        fps=100.0, double_arm=double, single_arm=not double,
        can_interface_left='can_left', can_interface_right='can_right',
        arx_single_arm_side=None if double else 'left', hardware_config={},
        use_start_position=False, start_position_path=None, start_position_move_duration=.02)


class YAMControllerTests(unittest.TestCase):
    def test_teaching_gripper_keeps_arm_passive_and_uses_force_limiter_gains(self):
        robot = FakeRobot()
        controller = YAMController('left', robot)
        try:
            with self.assertRaises(RuntimeError):
                controller.command_passive_gripper(0.)
            controller.enter_passive()
            for opening in (0., 1.):
                before = controller.read_state()[0]
                controller.command_passive_gripper(opening)
                command = robot.commands[-1]
                np.testing.assert_array_equal(command['kp'][:6], np.zeros(6))
                np.testing.assert_array_equal(command['kd'][:6], robot._grav_comp_kd[:6])
                np.testing.assert_array_equal(command['pos'][:6], before[:6])
                self.assertEqual(command['pos'][6], opening)
                self.assertEqual(command['kp'][6], robot._kp[6])
                self.assertEqual(command['kd'][6], robot._kd[6])
            count = len(robot.commands)
            for invalid in (-.1, 1.1, float('nan')):
                with self.assertRaises(ValueError):
                    controller.command_passive_gripper(invalid)
            self.assertEqual(len(robot.commands), count)
            controller.enter_safe_stop()
            with self.assertRaises(RuntimeError):
                controller.command_passive_gripper(0.)
        finally:
            controller.close()

    def test_modes_use_native_gains_and_hold_gripper_in_safe_stop(self):
        robot = FakeRobot()
        controller = YAMController('left', robot, command_timeout_s=2.0)
        try:
            target = np.r_[np.arange(6) / 10, .4]
            controller.command_target(target, kp_scale=.5, kd_scale=.75)
            np.testing.assert_allclose(robot.commands[-1]['pos'], target)
            np.testing.assert_allclose(robot.commands[-1]['kp'], robot._kp * .5)
            controller.enter_passive()
            self.assertEqual(robot.passive, 1)
            controller.enter_safe_stop()
            safe = robot.commands[-1]
            np.testing.assert_array_equal(safe['kp'][:6], np.zeros(6))
            self.assertEqual(safe['kp'][6], 5.0)
        finally:
            controller.close()
        self.assertEqual(robot.closed, 1)

    def test_command_watchdog_enters_safe_stop(self):
        robot = FakeRobot()
        controller = YAMController('left', robot, command_timeout_s=.04)
        try:
            controller.command_target(np.r_[np.zeros(6), 1.0])
            deadline = time.monotonic() + 1.0
            while controller._fault is None and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertIsNotNone(controller._fault)
            self.assertEqual(controller._mode, 'safe_stop')
            np.testing.assert_array_equal(robot.commands[-1]['kp'][:6], np.zeros(6))
        finally:
            controller.close()


class YAMBackendTests(unittest.TestCase):
    def test_teaching_gripper_only_commands_selected_role_in_passive_mode(self):
        backend, made = self.make_backend()
        try:
            with self.assertRaises(RuntimeError):
                backend.command_teaching_gripper('left', 0.)
            backend.enable_passive_compensation()
            backend.command_teaching_gripper('left', 0.)
            self.assertEqual(made['left'].gripper_commands, [0.])
            self.assertEqual(made['right'].gripper_commands, [])
            backend.command_teaching_gripper('right', 1.)
            self.assertEqual(made['right'].gripper_commands, [1.])
            with self.assertRaises(ValueError):
                backend.command_teaching_gripper('unknown', 0.)
            backend.hold_after_recording()
            with self.assertRaises(RuntimeError):
                backend.command_teaching_gripper('right', 0.)
        finally:
            backend.shutdown(home=False)

    def make_backend(self, double=True):
        made = {}
        def builder(interfaces, **options):
            made.update({role: FakeController(role) for role in interfaces})
            return made
        return YAMBackend(context(double), controller_builder=builder), made

    def test_dual_passive_record_and_synchronous_replay(self):
        backend, made = self.make_backend(double=True)
        try:
            backend.setup_for_recording()
            backend.enable_passive_compensation()
            self.assertEqual((made['left'].passive, made['right'].passive), (1, 1))
            backend.hold_after_recording()
            rows = np.vstack([
                np.r_[np.full(6, index / 100), .8, np.full(6, -index / 100), .7]
                for index in range(4)])
            self.assertTrue(backend.replay_rows(
                rows, fps=100, should_stop=lambda: False, kp_scale=.5, kd_scale=.75,
                sleep=lambda seconds: None))
            self.assertEqual(len(made['left'].commands), len(rows))
            self.assertEqual(len(made['right'].commands), len(rows))
            np.testing.assert_allclose(made['left'].commands[-1][0], rows[-1, :7])
            np.testing.assert_allclose(made['right'].commands[-1][0], rows[-1, 7:])
            self.assertEqual(made['left'].commands[-1][1:], (.5, .75))
        finally:
            backend.shutdown(home=False)
        self.assertEqual((made['left'].closed, made['right'].closed), (1, 1))

    def test_invalid_dual_row_is_rejected_before_first_command(self):
        backend, made = self.make_backend(double=True)
        try:
            backend.setup_for_recording()
            with self.assertRaises(ValueError):
                backend.replay_rows(np.zeros((2, 13)), fps=100, should_stop=lambda: False)
            self.assertFalse(made['left'].commands or made['right'].commands)
        finally:
            backend.shutdown(home=False)

    def test_factory_builds_inert_yam_adapter(self):
        config = {'type': 'yam', 'arms': {
            'left': {'model': 'yam_ultra_2', 'can_interface': 'can_left', 'gripper_force_n': 5.0}}}
        self.assertEqual(normalize_hardware_config(config)['type'], 'yam')
        arm = build_arms(config)['left']
        self.assertEqual(arm.describe().backend, 'yam')
        self.assertIsNone(arm._controller)
        arm.close()

    def test_factory_rejects_bad_yam_config_before_connect(self):
        with self.assertRaises(ValueError):
            build_arms({'type': 'yam', 'arms': {
                'left': {'model': 'yam_ultra_2', 'can_interface': 'can_left',
                         'gripper_force_n': 0.0}}})


if __name__ == '__main__':
    unittest.main()
