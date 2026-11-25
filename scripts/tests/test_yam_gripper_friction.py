"""Synthetic gripper coordinates/commands only; no CAN or robot construction."""
from dataclasses import replace
import json
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from hardware.backends.yam import YAMController
from hardware.yam_gripper_friction import GripperFriction, load_config, save_config


class Robot:
    def __init__(self, span=6.57):
        self._kp = np.array([80, 80, 80, 40, 10, 10, 20.])
        self._kd = np.array([5, 5, 5, 3, 1.5, 1.5, .5])
        self._grav_comp_kd = np.array([.1, .1, .1, .3, .05, .05, 0.])
        self._gripper_index = 6
        self._state_lock, self._command_lock = threading.Lock(), threading.Lock()
        self._joint_state = SimpleNamespace(pos=np.r_[np.zeros(6), .5],
                                            vel=np.zeros(7), timestamp=time.time())
        self.remapper = SimpleNamespace(to_robot_joint_vel_space=lambda v: np.r_[v[:6], v[6] * span])
        self.closed = False

    def command_joint_state(self, command):
        self._commands = SimpleNamespace(**{k: v.copy() for k, v in command.items()}, torques=np.zeros(7))

    def enter_gravity_comp_idle(self):
        self._commands = SimpleNamespace(pos=np.zeros(7), vel=np.zeros(7),
                                         kp=np.zeros(7), kd=self._grav_comp_kd.copy(), torques=np.zeros(7))

    def close(self):
        self.closed = True


@pytest.fixture
def make_controller(monkeypatch):
    # Drive watchdog ticks explicitly to verify transitions without sleep/races.
    monkeypatch.setattr('hardware.backends.yam.threading.Thread', lambda **kw: Mock())
    made = []
    def make(span=6.57):
        robot = Robot(span)
        clock = [10.]
        controller = YAMController('right', robot, clock=lambda: clock[0])
        controller.configure_gripper_friction(GripperFriction(coulomb_nm=.02))
        controller.enter_passive()
        robot._joint_state.vel[6] = .1
        made.append(controller)
        return controller, robot, clock
    yield make
    for controller in made:
        controller.close()


@pytest.mark.parametrize('span', [6.57, -6.57])
@pytest.mark.parametrize('velocity', [.1, -.1])
def test_normalized_velocity_is_mapped_once_and_only_j7_is_changed(make_controller, span, velocity):
    c, r, _ = make_controller(span)
    r._joint_state.vel[6] = velocity
    r._commands.torques[:6] = np.arange(6)
    gains = (r._commands.kp.copy(), r._commands.kd.copy())
    c._update_gripper_assist()
    assert np.sign(r._commands.torques[6]) == np.sign(velocity * span)
    assert abs(r._commands.torques[6]) <= .03
    np.testing.assert_array_equal(r._commands.torques[:6], np.arange(6))
    np.testing.assert_array_equal(r._commands.kp, gains[0])
    np.testing.assert_array_equal(r._commands.kd, gains[1])


def test_stationary_deadband_saturation_and_end_taper():
    p = GripperFriction(coulomb_nm=.06, viscous_nm_s_rad=.02)
    assert p.torque(.5, 0., 0.) == 0
    assert p.torque(.5, .001, .01) == 0
    full = p.torque(.5, .1, 1.)
    assert full == .03
    assert p.torque(.99, .1, 1.) == pytest.approx(.2 * full)
    assert p.torque(1., .1, 1.) == 0
    assert p.torque(1., -.1, -1.) == -full
    assert p.torque(-.01, -.1, -1.) == 0
    assert p.torque(.5, -.1, -1.) == -full


@pytest.mark.parametrize('values', [dict(coulomb_nm=float('nan')), dict(coulomb_nm=-.01),
    dict(max_torque_nm=.061), dict(transition_rad_s=0), dict(viscous_nm_s_rad=-.001),
    dict(deadband_rad_s=True), dict(end_margin=0), dict(max_speed_rad_s=100)])
def test_invalid_parameters_rejected(values):
    with pytest.raises(ValueError):
        GripperFriction(**values)


@pytest.mark.parametrize('mode', ['hold', 'replay', 'safe_stop', 'keyboard', 'close'])
def test_assist_cleared_on_every_nonmanual_transition(make_controller, mode):
    c, r, _ = make_controller()
    c._update_gripper_assist()
    assert r._commands.torques[6] != 0
    if mode == 'hold': c.hold_current()
    elif mode == 'replay': c.command_target(r._joint_state.pos)
    elif mode == 'safe_stop': c.enter_safe_stop()
    elif mode == 'keyboard': c.command_passive_gripper(.3)
    else: c.close()
    assert r._commands.torques[6] == 0
    if mode != 'close':
        c._update_gripper_assist()
        assert r._commands.torques[6] == 0


def test_tuning_holds_six_arm_joints_and_releases_only_gripper(make_controller):
    c, r, _ = make_controller()
    c.enter_gripper_tuning()
    np.testing.assert_array_equal(r._commands.kp[:6], r._kp[:6])
    np.testing.assert_array_equal(r._commands.kd[:6], r._kd[:6])
    assert r._commands.kp[6] == r._commands.kd[6] == 0
    c._update_gripper_assist()
    assert r._commands.torques[6] > 0


def test_heartbeat_expiry_removes_assist_without_changing_arm_commands(make_controller):
    c, r, clock = make_controller()
    c._update_gripper_assist()
    assert r._commands.torques[6] > 0
    clock[0] += .6
    c._update_gripper_assist()
    assert r._commands.torques[6] == 0
    c.check()
    c._update_gripper_assist()
    assert r._commands.torques[6] > 0
    c.configure_gripper_friction(c._gripper_friction, enabled=False)
    c._update_gripper_assist()
    assert r._commands.torques[6] == 0


@pytest.mark.parametrize('fault', ['stale', 'overspeed', 'nan'])
def test_worker_fault_clears_torque_and_surfaces_to_operator(make_controller, fault):
    c, r, _ = make_controller()
    c._update_gripper_assist()
    if fault == 'stale': r._joint_state.timestamp -= 1
    elif fault == 'overspeed': r._joint_state.vel[6] = 2.
    else: r._joint_state.vel[6] = float('nan')
    c._stop.wait = Mock(side_effect=[False, True])
    c._watch()
    assert r._commands.torques[6] == 0
    assert not c._gripper_assist_enabled
    with pytest.raises(RuntimeError, match='controller fault'):
        c.check()


def test_config_roundtrip_preserves_other_arm_and_rejects_arx(tmp_path):
    path = tmp_path / 'friction.json'
    left = GripperFriction(coulomb_nm=.01)
    right = replace(left, coulomb_nm=.015)
    save_config(path, 'left', left)
    save_config(path, 'right', right)
    assert load_config(path) == {'left': left, 'right': right}
    document = json.loads(path.read_text())
    document['robot'] = 'arx'
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError): load_config(path)


def test_sdk_shutdown_still_runs_if_assist_clear_fails(make_controller):
    c, r, _ = make_controller()
    r._commands.torques = None
    with pytest.raises(TypeError):
        c.close()
    assert r.closed
