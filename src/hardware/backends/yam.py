"""YAM Ultra 2 support through the i2rt SDK.

The public coordinates are six joint angles in radians followed by the
normalized linear-gripper position (0 closed, 1 open).  Construction is lazy:
importing this module never opens CAN or starts an i2rt control thread.
"""
from __future__ import annotations

import os
from pathlib import Path
import sys
import threading
import time
from typing import Any, Callable

import numpy as np

from hardware.arm_interface import ArmDescription, ArmState, ManagedArm
from hardware.yam_gripper_friction import AccelerationEstimator


ARM_DOF = 6
ACTION_DIM = 7
DEFAULT_GRIPPER_FORCE_N = 5.0
DEFAULT_MOVE_HZ = 100.0
DEFAULT_MOVE_DURATION_S = 3.0
DEFAULT_MAX_TEMPERATURE_C = 70.0


def ensure_i2rt_on_path(root: str | os.PathLike[str] | None = None) -> Path | None:
    """Make a source checkout importable, while also accepting installed i2rt."""
    candidates = []
    if root:
        candidates.append(Path(root).expanduser())
    if os.environ.get("I2RT_ROOT"):
        candidates.append(Path(os.environ["I2RT_ROOT"]).expanduser())
    # This repository is normally cloned beside the i2rt checkout.
    candidates.append(Path(__file__).resolve().parents[4] / "i2rt")
    for candidate in candidates:
        if (candidate / "i2rt" / "robots" / "get_robot.py").is_file():
            resolved = candidate.resolve()
            if str(resolved) not in sys.path:
                sys.path.insert(0, str(resolved))
            return resolved
    try:
        import i2rt  # noqa: F401
    except ImportError as exc:
        locations = ", ".join(str(path) for path in candidates)
        raise ImportError(
            "i2rt is not importable; set I2RT_ROOT to its checkout "
            f"(checked: {locations})"
        ) from exc
    return None


def _i2rt_types(root=None):
    ensure_i2rt_on_path(root)
    from i2rt.robots.get_robot import get_yam_robot, get_yam_robots
    from i2rt.robots.utils import ArmType, GripperForceLimiter, GripperType
    return get_yam_robot, get_yam_robots, ArmType, GripperType, GripperForceLimiter


def _complete_state(robot: Any) -> tuple[np.ndarray, np.ndarray, float]:
    """Copy qpos, qvel and publication time from one i2rt state object."""
    lock = getattr(robot, "_state_lock", None)
    native = getattr(robot, "_joint_state", None)
    if lock is not None and native is not None:
        with lock:
            native = robot._joint_state
            return (np.asarray(native.pos, dtype=float).copy(),
                    np.asarray(native.vel, dtype=float).copy(), float(native.timestamp))
    # Test doubles and alternate SDK implementations can expose this public pair.
    state = robot.get_joint_state()
    return (np.asarray(state["pos"], dtype=float).copy(),
            np.asarray(state["vel"], dtype=float).copy(), time.time())


def _check_health(robot: Any, *, max_temperature_c: float) -> None:
    chain = getattr(robot, "motor_chain", None)
    if chain is None:
        return
    states = chain.read_states()
    bad = [(index + 1, state.error_code) for index, state in enumerate(states)
           if state.error_code != "0x1"]
    if bad:
        raise RuntimeError(f"YAM motor error(s): {bad}")
    temperatures = [max(float(state.temp_mos), float(state.temp_rotor)) for state in states]
    if temperatures and max(temperatures) >= max_temperature_c:
        raise RuntimeError(
            f"YAM temperature {max(temperatures):.1f} C >= {max_temperature_c:.1f} C"
        )


def _configure_force_limit(robot: Any, force_n: float, force_limiter_type: Any,
                           gripper_type: Any, arm_type: Any) -> None:
    force_n = float(force_n)
    if not np.isfinite(force_n) or not 0 < force_n <= 25:
        raise ValueError("YAM gripper force must be finite and in (0, 25] N")
    # The SDK builder defaults to 50 N.  This project uses a bottle-safe limit.
    robot._gripper_force_limiter = force_limiter_type(
        max_force=force_n, gripper_type=gripper_type, arm_type=arm_type,
        kp=float(robot._kp[ARM_DOF]))
    robot._limit_gripper_force = force_n


class YAMController:
    """One connected YAM with passive, hold, replay and safe-stop modes."""

    def __init__(self, role: str, robot: Any, *, gripper_force_n=DEFAULT_GRIPPER_FORCE_N,
                 max_temperature_c=DEFAULT_MAX_TEMPERATURE_C, command_timeout_s=0.5,
                 clock=time.monotonic):
        self.role, self.robot = role, robot
        self.gripper_force_n = float(gripper_force_n)
        self.max_temperature_c = float(max_temperature_c)
        self.command_timeout_s = float(command_timeout_s)
        self.clock = clock
        self._gripper_friction = None
        self._gripper_assist_enabled = False
        self._gripper_position_control = True
        self._assist_deadline = None
        self._assist_torque = 0.0
        self._assist_acceleration = AccelerationEstimator()
        self._assist_pulse = None
        self._pulse_rearm_at = 0.0
        self._assist_stats = dict(ticks=0, peak_motor_speed_rad_s=0., peak_acceleration_rad_s2=0.,
                                  peak_raw_nm=0., peak_command_nm=0., capped_ticks=0)
        self._base_kp = np.asarray(robot._kp, dtype=float).copy()
        self._base_kd = np.asarray(robot._kd, dtype=float).copy()
        self._passive_kd = np.asarray(robot._grav_comp_kd, dtype=float).copy()
        self._startup = self.read_state()[0]
        self._mode = "hold"
        self._deadline: float | None = None
        self._fault: BaseException | None = None
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._watchdog = threading.Thread(
            target=self._watch, name=f"yam-{role}-command-watchdog", daemon=True)
        self.hold_current()
        self._watchdog.start()

    def _require_live(self) -> None:
        if self._stop.is_set():
            raise RuntimeError(f"YAM {self.role} controller is closed")
        if self._fault is not None:
            raise RuntimeError(f"YAM {self.role} controller fault: {self._fault}")

    def read_state(self) -> tuple[np.ndarray, np.ndarray, float]:
        _check_health(self.robot, max_temperature_c=self.max_temperature_c)
        qpos, qvel, timestamp = _complete_state(self.robot)
        if qpos.shape != (ACTION_DIM,) or qvel.shape != (ACTION_DIM,):
            raise RuntimeError(f"YAM {self.role} expected seven feedback values")
        if not np.isfinite(qpos).all() or not np.isfinite(qvel).all():
            raise RuntimeError(f"YAM {self.role} returned non-finite feedback")
        if self._gripper_assist_enabled:
            self._assist_deadline = self.clock() + self.command_timeout_s
        return qpos, qvel, timestamp

    @staticmethod
    def _target(value: Any) -> np.ndarray:
        target = np.asarray(value, dtype=float).copy()
        if target.shape != (ACTION_DIM,) or not np.isfinite(target).all():
            raise ValueError("YAM target must contain seven finite values")
        if not 0.0 <= target[ARM_DOF] <= 1.0:
            raise ValueError("YAM normalized gripper target must be in [0, 1]")
        return target

    def _command(self, target: np.ndarray, kp: np.ndarray, kd: np.ndarray) -> None:
        self._clear_gripper_assist()
        self._gripper_position_control = True
        self.robot.command_joint_state({
            "pos": target, "vel": np.zeros(ACTION_DIM), "kp": kp, "kd": kd})

    def command_target(self, target: Any, *, kp_scale=1.0, kd_scale=1.0) -> None:
        with self._lock:
            self._require_live()
            target = self._target(target)
            self._command(target, self._base_kp * float(kp_scale), self._base_kd * float(kd_scale))
            self._mode = "replay"
            self._deadline = self.clock() + self.command_timeout_s

    def hold_current(self) -> np.ndarray:
        with self._lock:
            self._require_live()
            target = self.read_state()[0]
            self._command(target, self._base_kp, self._base_kd)
            self._mode, self._deadline = "hold", None
            return target

    def enter_passive(self) -> None:
        with self._lock:
            self._require_live()
            self._clear_gripper_assist()
            self.robot.enter_gravity_comp_idle()
            self._gripper_position_control = False
            self._assist_torque = 0.0
            self._mode, self._deadline = "passive", None

    def configure_gripper_friction(self, parameters, *, enabled=True):
        """Opt-in only; preserve arm dynamics and all position/replay gains."""
        from hardware.yam_gripper_friction import GripperFriction
        if not isinstance(parameters, GripperFriction):
            raise TypeError('Expected validated GripperFriction parameters')
        if (getattr(self.robot, '_gripper_index', None) != ARM_DOF
                or not hasattr(self.robot, 'remapper')
                or not hasattr(self.robot, '_command_lock')):
            raise RuntimeError('Gripper assist requires the pinned i2rt linear-gripper interface')
        with self._lock:
            self._require_live()
            self._clear_gripper_assist()
            self._gripper_friction = parameters
            self._gripper_assist_enabled = bool(enabled)
            self._assist_deadline = self.clock() + self.command_timeout_s

    def enter_gripper_tuning(self):
        """Hold the measured arm pose; free only the gripper for manual tuning."""
        with self._lock:
            self._require_live()
            target = self.read_state()[0]
            kp, kd = self._base_kp.copy(), self._base_kd.copy()
            kp[ARM_DOF] = kd[ARM_DOF] = 0.0
            self._command(target, kp, kd)
            self._gripper_position_control = False
            self._mode, self._deadline = 'gripper_tuning', None

    def _clear_gripper_assist(self):
        self._assist_pulse = None
        self._assist_acceleration.reset()
        if self._gripper_friction is not None:
            with self.robot._command_lock:
                self.robot._commands.torques[ARM_DOF] = 0.0
        self._assist_torque = 0.0

    def request_gripper_pulse(self, opening_direction, magnitude_nm):
        """Operator-triggered 150ms pulse; diagnostic tuning only, never recording."""
        with self._lock:
            self._require_live()
            if (self._mode != 'gripper_tuning' or self._gripper_position_control
                    or not self._gripper_assist_enabled):
                raise RuntimeError('Pulse requires gripper tuning with master assistance ON')
            if (opening_direction not in (-1, 1) or not np.isfinite(magnitude_nm)
                    or not 0 < magnitude_nm <= self._gripper_friction.max_torque_nm):
                raise ValueError('Invalid pulse direction or magnitude')
            now = self.clock()
            if now < self._pulse_rearm_at:
                return False
            self._assist_pulse = (opening_direction, magnitude_nm, now + .15)
            self._pulse_rearm_at = now + .5
            return True

    def gripper_assist_diagnostics(self):
        """Drain aggregate control statistics; no separate observation/trajectory stream."""
        with self._lock:
            result = dict(self._assist_stats)
            for key in self._assist_stats:
                self._assist_stats[key] = 0
            return result

    def _update_gripper_assist(self):
        """Called under controller lock by the existing 100 Hz watchdog thread."""
        if self._gripper_friction is None:
            return
        if (not self._gripper_assist_enabled or self._gripper_position_control
                or self._mode not in ('passive', 'gripper_tuning')
                or self._assist_deadline is None or self.clock() >= self._assist_deadline):
            self._clear_gripper_assist()
            return
        # Separate control feedback, never reused as a recorded observation.
        # Do not renew the main-thread heartbeat from the assistance worker.
        _check_health(self.robot, max_temperature_c=self.max_temperature_c)
        q, dq, timestamp = _complete_state(self.robot)
        if not np.isfinite(timestamp) or not 0 <= time.time() - timestamp <= 0.1:
            raise RuntimeError('Gripper assist feedback is stale or has an invalid clock')
        motor_velocity = float(self.robot.remapper.to_robot_joint_vel_space(dq)[ARM_DOF])
        parameters = self._gripper_friction
        acceleration = (self._assist_acceleration.update(
            motor_velocity, timestamp, parameters.acceleration_filter_s)
            if parameters.acceleration_enabled else 0.0)
        opening_span = float(self.robot.remapper.to_robot_joint_vel_space(np.r_[np.zeros(6), 1.])[ARM_DOF])
        if not np.isfinite(opening_span) or opening_span == 0:
            raise RuntimeError('Invalid calibrated gripper direction')
        opening_sign = float(np.sign(opening_span))
        pulse_nm = 0.
        if self._assist_pulse is not None:
            direction, magnitude, until = self._assist_pulse
            if self.clock() < until:
                pulse_nm = direction * opening_sign * magnitude
            else:
                self._assist_pulse = None
        diagnostic = {}
        torque = parameters.torque(float(q[ARM_DOF]), float(dq[ARM_DOF]), motor_velocity, acceleration,
                                  pulse_nm=pulse_nm, motor_opening_sign=opening_sign, diagnostics=diagnostic)
        with self.robot._command_lock:
            # The pinned public command_joint_state ignores a "torques" field.
            # Use its existing command lock and touch ONLY J7 feedforward.
            command = self.robot._commands
            if command.kp[ARM_DOF] != 0:
                torque = 0.0
                self._assist_pulse = None
                self._assist_acceleration.reset()
            command.torques[ARM_DOF] = torque
        self._assist_torque = torque
        stats = self._assist_stats
        stats['ticks'] += 1
        stats['capped_ticks'] += int(diagnostic['capped'])
        for key, value in (('peak_motor_speed_rad_s', motor_velocity),
                           ('peak_acceleration_rad_s2', acceleration),
                           ('peak_raw_nm', diagnostic['raw_nm']), ('peak_command_nm', torque)):
            stats[key] = max(stats[key], abs(value))

    def command_passive_gripper(self, opening: float) -> None:
        """Position the gripper while leaving all six arm joints hand-guided.

        Uses the native gripper gains matching the configured SDK force limiter.
        The SDK maintains the command; no new robot polling/control loop is added.
        """
        opening = float(opening)
        if not np.isfinite(opening) or not 0 <= opening <= 1:
            raise ValueError("Gripper opening must be in [0, 1]")
        with self._lock:
            self._require_live()
            if self._mode != "passive":
                raise RuntimeError("Teaching gripper commands require passive arm mode")
            target = self.read_state()[0].copy()
            target[ARM_DOF] = opening
            kp = np.zeros(ACTION_DIM)
            kd = self._passive_kd.copy()
            kp[ARM_DOF] = self._base_kp[ARM_DOF]
            kd[ARM_DOF] = self._base_kd[ARM_DOF]
            self._command(target, kp, kd)

    def enter_safe_stop(self) -> None:
        """Gravity support with zero arm Kp; hold only the measured gripper."""
        with self._lock:
            if self._stop.is_set():
                return
            qpos = self.read_state()[0]
            kp = np.zeros(ACTION_DIM)
            kd = self._passive_kd.copy()
            kp[ARM_DOF] = min(5.0, self._base_kp[ARM_DOF])
            kd[ARM_DOF] = self._base_kd[ARM_DOF]
            self._command(qpos, kp, kd)
            self._mode, self._deadline = "safe_stop", None

    def check(self) -> None:
        with self._lock:
            self._require_live()
            self.read_state()

    practice_step = check

    def move_to(self, target: Any, *, duration=DEFAULT_MOVE_DURATION_S,
                command_hz=DEFAULT_MOVE_HZ, smooth=True) -> None:
        target = self._target(target)
        duration, command_hz = float(duration), float(command_hz)
        if not np.isfinite(duration) or duration <= 0 or not np.isfinite(command_hz) or command_hz <= 0:
            raise ValueError("YAM move duration and command rate must be positive and finite")
        start = self.read_state()[0]
        steps = max(2, int(round(duration * command_hz)))
        dt = duration / steps
        next_tick = self.clock()
        for index in range(1, steps + 1):
            alpha = index / steps
            if smooth:
                alpha = 10 * alpha**3 - 15 * alpha**4 + 6 * alpha**5
            self.command_target(start + alpha * (target - start))
            next_tick += dt
            time.sleep(max(0.0, next_tick - self.clock()))
        with self._lock:
            self._mode, self._deadline = "hold", None

    def go_home(self) -> None:
        self._fault = None
        self.move_to(self._startup, duration=DEFAULT_MOVE_DURATION_S, smooth=True)

    def _watch(self) -> None:
        while not self._stop.wait(0.01):
            with self._lock:
                try:
                    self._update_gripper_assist()
                except Exception as exc:
                    self._gripper_assist_enabled = False
                    self._fault = exc
                    self._clear_gripper_assist()
                if self._mode == "replay" and self._deadline is not None and self.clock() >= self._deadline:
                    try:
                        self.enter_safe_stop()
                    except BaseException as exc:
                        self._fault = exc
                    else:
                        self._fault = RuntimeError("YAM replay command expired; entered safe stop")

    def close(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        if threading.current_thread() is not self._watchdog:
            self._watchdog.join(timeout=1.0)
        try:
            with self._lock:
                self._clear_gripper_assist()
        finally:
            self.robot.close()


def create_yam_controllers(interfaces: dict[str, str], *, i2rt_root=None,
                           gripper_force_n=DEFAULT_GRIPPER_FORCE_N,
                           max_temperature_c=DEFAULT_MAX_TEMPERATURE_C,
                           robot_builder: Callable[..., list[Any]] | None = None) -> dict[str, YAMController]:
    """Construct all selected arms together and close partial construction on failure."""
    _, get_many, arm_type, gripper_type, limiter = _i2rt_types(i2rt_root)
    builder = robot_builder or get_many
    robots = []
    controllers: dict[str, YAMController] = {}
    try:
        robots = builder(
            arms=[{"channel": interface} for interface in interfaces.values()],
            arm_type=arm_type.YAM_ULTRA_2,
            gripper_type=gripper_type.LINEAR_4310,
            zero_gravity_mode=False,
            enable_auto_recovery=False,
            use_coulomb_friction=False,
        )
        if len(robots) != len(interfaces):
            raise RuntimeError("i2rt returned the wrong number of YAM controllers")
        for (role, _), robot in zip(interfaces.items(), robots, strict=True):
            _configure_force_limit(robot, gripper_force_n, limiter,
                                   gripper_type.LINEAR_4310, arm_type.YAM_ULTRA_2)
            controllers[role] = YAMController(
                role, robot, gripper_force_n=gripper_force_n,
                max_temperature_c=max_temperature_c)
        return controllers
    except BaseException:
        for controller in controllers.values():
            try:
                controller.close()
            except Exception:
                pass
        controlled = {id(controller.robot) for controller in controllers.values()}
        for robot in robots:
            if id(robot) not in controlled:
                try:
                    robot.close()
                except Exception:
                    pass
        raise


class YAMDamping:
    """safe_exit target for connected YAM controllers."""

    def __init__(self, controllers: list[YAMController], *, before_home=None):
        self.controllers = controllers
        self.before_home = before_home

    def damp(self):
        for controller in self.controllers:
            controller.enter_safe_stop()

    def refresh(self):
        for controller in self.controllers:
            # A watchdog fault is the reason safe_exit may be running; feedback
            # must remain readable while the operator decides to home.
            controller.read_state()

    def go_home(self):
        if self.before_home is not None:
            self.before_home()
        for controller in self.controllers:
            controller.go_home()


class YAMBackend:
    """Flash hand-guided record/replay backend for one or two YAM Ultra 2 arms."""

    keyboard_start = True

    def __init__(self, ctx: Any, *, controller_builder=create_yam_controllers):
        self.ctx = ctx
        if not np.isfinite(ctx.fps) or not 0 < ctx.fps <= 200:
            raise ValueError("YAM recording rate must be in (0, 200] Hz")
        if ctx.double_arm:
            interfaces = {"left": ctx.can_interface_left or "can_left",
                          "right": ctx.can_interface_right or "can_right"}
        else:
            role = ctx.arx_single_arm_side or "left"
            interfaces = {role: ctx.can_interface_left or ("can_right" if role == "right" else "can_left")}
        options = dict(ctx.hardware_config or {})
        options.pop("type", None)
        options.pop("arms", None)
        self._by_role = controller_builder(interfaces, **options)
        self.controllers = list(self._by_role.values())
        self.start_position_applied = False
        self.before_home = None
        self.mode: str | None = None
        self._closed = False
        print("[YAM] Recording in native joint radians and normalized linear-gripper position")

    def _require_open(self):
        if self._closed:
            raise RuntimeError("YAM recorder is closed")

    def _split(self, qpos):
        return {role: np.asarray(qpos[index * ACTION_DIM:(index + 1) * ACTION_DIM], dtype=float).copy()
                for index, role in enumerate(self._by_role)}

    def _require_pose(self, qpos, label):
        qpos = np.asarray(qpos, dtype=float)
        if qpos.shape != (ACTION_DIM * len(self._by_role),) or not np.isfinite(qpos).all():
            raise ValueError(f"{label} must contain seven finite positions per selected arm")
        if np.any((qpos[ARM_DOF::ACTION_DIM] < 0) | (qpos[ARM_DOF::ACTION_DIM] > 1)):
            raise ValueError(f"{label} gripper positions must be in [0, 1]")
        return qpos

    def setup_for_recording(self):
        self._require_open()
        self.start_position_applied = False
        if self.ctx.use_start_position:
            from recording.record_episode_single import load_start_qpos, resolve_repo_path
            target = load_start_qpos(self.ctx.start_position_path, self.ctx.single_arm,
                                     next(iter(self._by_role)) if self.ctx.single_arm else None)
            print(f"[YAM] Moving to saved start pose: {resolve_repo_path(self.ctx.start_position_path)}")
            self.move_to_pose(target)
            self.start_position_applied = True
        else:
            self.enter_motion_mode()
        print("[YAM] Motion mode: holding arms and grippers")

    def move_to_pose(self, qpos):
        self._require_open()
        targets = self._split(self._require_pose(qpos, "YAM pose"))
        # Move both arms concurrently, one command per arm per 100 Hz tick.
        starts = {role: controller.read_state()[0] for role, controller in self._by_role.items()}
        duration = float(self.ctx.start_position_move_duration)
        steps = max(2, int(round(duration * DEFAULT_MOVE_HZ)))
        next_tick = time.monotonic()
        for index in range(1, steps + 1):
            alpha = index / steps
            alpha = 10 * alpha**3 - 15 * alpha**4 + 6 * alpha**5
            for role, controller in self._by_role.items():
                controller.command_target(starts[role] + alpha * (targets[role] - starts[role]))
            next_tick += duration / steps
            time.sleep(max(0.0, next_tick - time.monotonic()))
        for controller in self.controllers:
            with controller._lock:
                controller._mode, controller._deadline = "hold", None
        self.mode = "motion"

    def replay_rows(self, actions, *, fps, should_stop, kp_scale=1.0, kd_scale=1.0,
                    clock=time.monotonic, sleep=time.sleep, log=None, on_tick=None):
        self._require_open()
        if self.mode != "motion":
            raise RuntimeError("Replay starts from motion mode at the recorded start pose")
        actions = np.asarray(actions, dtype=float)
        if actions.ndim != 2 or actions.shape[1] != ACTION_DIM * len(self._by_role):
            raise ValueError("Replay rows must contain seven positions per selected arm")
        if not np.isfinite(actions).all():
            raise ValueError("Replay rows must be finite")
        kp_scale, kd_scale = float(kp_scale), float(kd_scale)
        if not 0 < kp_scale <= 2 or not 0 <= kd_scale <= 2:
            raise ValueError("YAM replay gain scales require kp in (0, 2] and kd in [0, 2]")
        dt = 1.0 / float(fps)
        if not np.isfinite(dt) or not 0.005 <= dt <= 0.2:
            raise ValueError("YAM replay rate must be in [5, 200] Hz")
        for index, row in enumerate(actions):
            if should_stop():
                return False
            started = clock()
            if on_tick is not None:
                on_tick(index, row)
            for role, target in self._split(row).items():
                self._by_role[role].command_target(target, kp_scale=kp_scale, kd_scale=kd_scale)
            if log is not None:
                log.step(row=index, target=row.copy(), qpos=self.read_qpos())
            remaining = dt - (clock() - started)
            if remaining > 0:
                sleep(remaining)
        return True

    def enter_safe_stop(self):
        for controller in self.controllers:
            controller.enter_safe_stop()
        self.mode = "safe_stop"
        print("[YAM] Safe stop: gravity support, zero arm Kp; grippers held")

    def enter_motion_mode(self):
        for controller in self.controllers:
            controller.hold_current()
        self.mode = "motion"

    def enable_passive_compensation(self):
        for controller in self.controllers:
            controller.enter_passive()
        self.mode = "passive"
        print("[YAM] Passive mode active: arm/gripper Kp=0, gravity support on")

    enter_recording_mode = enable_passive_compensation

    def start_recording(self):
        self.enter_recording_mode()

    def abort_recording(self):
        self.hold_after_recording()

    def read_state(self):
        states = []
        for controller in self.controllers:
            # Surface an assistance/watchdog fault without taking a second SDK snapshot.
            if isinstance(controller, YAMController):
                controller._require_live()
            states.append(controller.read_state())
        return (np.concatenate([state[0] for state in states]).astype(np.float32),
                np.concatenate([state[1] for state in states]).astype(np.float32))

    def read_qpos(self):
        return self.read_state()[0]

    def read_qvel(self):
        return self.read_state()[1]

    def passive_step(self):
        for controller in self.controllers:
            controller.check()

    def command_teaching_gripper(self, role, opening):
        self._require_open()
        if self.mode != "passive":
            raise RuntimeError("Teaching gripper commands require passive arm mode")
        if role not in self._by_role:
            raise ValueError(f"No YAM arm for role {role!r}")
        self._by_role[role].command_passive_gripper(opening)

    def hold_after_recording(self):
        self.enter_motion_mode()
        print("[YAM] Motion mode: holding measured arm and gripper poses for review")

    def damping_target(self):
        return YAMDamping(self.controllers, before_home=self.before_home)

    def shutdown(self, *, home=True):
        if self._closed:
            return
        if home:
            print("[YAM] Returning arm(s) to their startup poses")
            for controller in self.controllers:
                controller.go_home()
        self._closed = True
        errors = []
        for controller in self.controllers:
            try:
                controller.close()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError("One or more YAM controllers failed to close") from errors[0]


class YAMAdapter(ManagedArm):
    """Shared Flash hardware adapter used by task-configured runtimes."""

    def __init__(self, role, model, can_interface, *, i2rt_root=None,
                 gripper_force_n=DEFAULT_GRIPPER_FORCE_N, robot_builder=None, **options):
        if model != "yam_ultra_2":
            raise ValueError(f"Unsupported YAM model {model!r}")
        if not np.isfinite(float(gripper_force_n)) or not 0 < float(gripper_force_n) <= 25:
            raise ValueError("YAM gripper_force_n must be finite and in (0, 25]")
        self._can, self._root = can_interface, i2rt_root
        self._force = float(gripper_force_n)
        self._builder = robot_builder
        self._controller: YAMController | None = None
        super().__init__(ArmDescription(role, model, "yam", ACTION_DIM, 200.0), **options)

    def _connect(self):
        controllers = create_yam_controllers(
            {self.describe().role: self._can}, i2rt_root=self._root,
            gripper_force_n=self._force, robot_builder=self._builder)
        self._controller = controllers[self.describe().role]

    def _read_state(self):
        qpos, qvel, wall_time = self._controller.read_state()
        age = max(0.0, time.time() - wall_time)
        host_time = self._clock() - age
        return ArmState(qpos, qvel, np.full(ACTION_DIM, age), np.full(ACTION_DIM, host_time))

    def _validate_limits(self, target):
        if not 0 <= target[ARM_DOF] <= 1:
            raise ValueError("YAM gripper target must be in [0, 1]")

    def _submit_target(self, target, *, interval_s):
        self._controller.command_target(target)

    def _submit_safe_stop(self, *, interval_s):
        self._controller.enter_safe_stop()

    def open_gripper_target(self):
        return 1.0

    def _home(self):
        self._controller.go_home()

    def _close(self):
        if self._controller is not None:
            controller, self._controller = self._controller, None
            controller.close()
