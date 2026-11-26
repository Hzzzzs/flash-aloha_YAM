"""Guarded first gravity-compensation run using the shipped Ultra2 + linear_4310 model."""

from __future__ import annotations

import time

import numpy as np
import tyro

from i2rt.motor_drivers.dm_driver import ControlMode
from i2rt.motor_drivers.motor_check import run_startup_checks
from i2rt.motor_drivers.utils import MotorType
from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import ArmType, GripperType


ULTRA2_WITH_GRIPPER = [
    [1, MotorType.DM4340],
    [2, MotorType.DM4340],
    [3, MotorType.DM4340],
    [4, MotorType.DM4340],
    [5, MotorType.DM4310],
    [6, MotorType.DM4310],
    [7, MotorType.DM4310],
]

# Placeholder limits only prevent the linear gripper's automatic end-stop calibration. The gripper's
# gains are zero throughout this probe, so these limits are never used to command a motion.
NO_CALIBRATION_LIMITS = np.array([-3.3, 3.3])


def main(
    channel: str,
    duration_s: float = 3.0,
    max_displacement_rad: float = 0.08,
    max_velocity_rad_s: float = 0.35,
    max_temperature_c: float = 60.0,
    max_torque_nm: float = 20.0,
) -> None:
    """Run a guarded, short gravity-compensation trial with the repository default end-effector model.

    Args:
        channel: Exactly one of can_left or can_right.
        duration_s: Free gravity-compensation duration; restricted to 2–20 seconds.
        max_displacement_rad: Abort if any arm joint drifts this far after entering gravity compensation.
        max_velocity_rad_s: Abort if any arm joint exceeds this reported speed.
        max_temperature_c: Abort if a MOS or rotor temperature reaches this value.
        max_torque_nm: Abort if any computed arm motor torque exceeds this magnitude.
    """
    if channel not in {"can_left", "can_right"}:
        raise SystemExit("channel must be can_left or can_right")
    if not 2.0 <= duration_s <= 20.0:
        raise SystemExit("duration-s must be between 2 and 20 seconds")

    print("Running read-only startup survey for joints 1-6 and gripper ID 7...", flush=True)
    run_startup_checks(
        channel,
        ULTRA2_WITH_GRIPPER,
        check_motor_types=True,
        check_motor_config=True,
        control_mode=ControlMode.MIT,
        repair=False,
    )

    print("Building yam_ultra_2 + linear_4310 with repository default dynamics.", flush=True)
    print("D405, bracket and cable mass are NOT added to the model.", flush=True)
    robot = get_yam_robot(
        channel=channel,
        arm_type=ArmType.YAM_ULTRA_2,
        gripper_type=GripperType.LINEAR_4310,
        zero_gravity_mode=False,
        gripper_limits_override=NO_CALIBRATION_LIMITS,
        gripper_kp=0.0,
        gripper_kd=0.0,
        enable_auto_recovery=False,
        use_coulomb_friction=False,
    )

    try:
        factors = robot.gravity_comp_factor.copy()
        print(f"gravity_comp_factor={np.array2string(factors[:6], precision=3)}", flush=True)
        print("Stabilising for 1 second in current-pose PD hold plus gravity feedforward...", flush=True)
        time.sleep(1.0)

        pre_states = robot.motor_chain.read_states()
        bad_pre = [(index + 1, state.error_code) for index, state in enumerate(pre_states) if state.error_code != "0x1"]
        if bad_pre:
            raise RuntimeError(f"motor error(s) before gravity-comp idle: {bad_pre}")

        robot.enter_gravity_comp_idle()
        q0 = robot.get_joint_pos()[:6].copy()
        print(f"entered_gravity_comp reference_q_rad={np.array2string(q0, precision=5)}", flush=True)

        started = time.monotonic()
        next_report = started
        max_seen_displacement = 0.0
        max_seen_velocity = 0.0
        max_seen_torque = 0.0

        while True:
            now = time.monotonic()
            if now - started >= duration_s:
                break

            states = robot.motor_chain.read_states()
            q = robot.get_joint_pos()[:6].copy()
            velocities = np.array([state.vel for state in states[:6]], dtype=float)
            mos = np.array([state.temp_mos for state in states], dtype=float)
            rotor = np.array([state.temp_rotor for state in states], dtype=float)
            torques = robot.get_motor_torques()
            arm_torques = np.asarray(torques[:6], dtype=float) if torques is not None else np.zeros(6)

            displacement = float(np.max(np.abs(q - q0)))
            speed = float(np.max(np.abs(velocities)))
            torque = float(np.max(np.abs(arm_torques)))
            temperature = float(max(np.max(mos), np.max(rotor)))
            max_seen_displacement = max(max_seen_displacement, displacement)
            max_seen_velocity = max(max_seen_velocity, speed)
            max_seen_torque = max(max_seen_torque, torque)

            bad_errors = [(index + 1, state.error_code) for index, state in enumerate(states) if state.error_code != "0x1"]
            if bad_errors:
                raise RuntimeError(f"motor error(s): {bad_errors}")
            if displacement > max_displacement_rad:
                raise RuntimeError(
                    f"drift guard tripped: displacement {displacement:.5f} rad > {max_displacement_rad:.5f} rad"
                )
            if speed > max_velocity_rad_s:
                raise RuntimeError(f"velocity guard tripped: {speed:.5f} rad/s > {max_velocity_rad_s:.5f} rad/s")
            if temperature >= max_temperature_c:
                raise RuntimeError(f"temperature guard tripped: {temperature:.1f} C >= {max_temperature_c:.1f} C")
            if torque > max_torque_nm:
                raise RuntimeError(f"torque guard tripped: {torque:.3f} Nm > {max_torque_nm:.3f} Nm")

            if now >= next_report:
                print(
                    f"t={now - started:.2f}s q={np.array2string(q, precision=4)} "
                    f"max_dq={speed:.4f} torque_Nm={np.array2string(arm_torques, precision=3)} "
                    f"mos_C={mos.astype(int).tolist()} rotor_C={rotor.astype(int).tolist()}",
                    flush=True,
                )
                next_report += 1.0
            time.sleep(0.02)

        print(
            f"PASS default gravity compensation: duration={duration_s:.1f}s "
            f"max_displacement={max_seen_displacement:.5f}rad "
            f"max_velocity={max_seen_velocity:.5f}rad/s max_torque={max_seen_torque:.3f}Nm",
            flush=True,
        )
    finally:
        print("Closing motors; continue physically supporting the arm as it becomes limp.", flush=True)
        robot.close()


if __name__ == "__main__":
    tyro.cli(main)
