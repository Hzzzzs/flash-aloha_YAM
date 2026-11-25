"""Interactive multi-pose static drift evaluation for YAM Ultra2 gravity compensation."""

from __future__ import annotations

import csv
import json
import select
import signal
import sys
import time
from pathlib import Path

import numpy as np
import tyro

from i2rt.motor_drivers.dm_driver import ControlMode
from i2rt.motor_drivers.motor_check import run_startup_checks
from i2rt.motor_drivers.utils import MotorType
from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.kinematics import Kinematics
from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml


ULTRA2_WITH_GRIPPER = [
    [1, MotorType.DM4340],
    [2, MotorType.DM4340],
    [3, MotorType.DM4340],
    [4, MotorType.DM4340],
    [5, MotorType.DM4310],
    [6, MotorType.DM4310],
    [7, MotorType.DM4310],
]
NO_CALIBRATION_LIMITS = np.array([-3.3, 3.3])
# The arm's existing position-control gains; the gripper remains torque-free.
DIAGNOSTIC_HOLD_KP = np.array([80.0, 80.0, 80.0, 40.0, 10.0, 10.0, 0.0])
DIAGNOSTIC_HOLD_KD = np.array([5.0, 5.0, 5.0, 3.0, 1.5, 1.5, 0.0])


def _close_robot(robot) -> None:
    # A second Ctrl+C must not interrupt the sequence that disables all motors.
    previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        try:
            print("Closing motors; keep supporting the arm as it becomes limp.", flush=True)
        except OSError:
            # A closed log pipe must never bypass motor shutdown.
            pass
        robot.close()
    finally:
        signal.signal(signal.SIGINT, previous_handler)


def _tool_pose(kinematics: Kinematics, arm_q: np.ndarray) -> np.ndarray:
    # The two finger-slide positions do not change grasp_site; zeros are sufficient for FK.
    return kinematics.fk(np.concatenate([arm_q, np.zeros(2)]), "grasp_site")


def _rotation_error_deg(reference: np.ndarray, current: np.ndarray) -> float:
    relative = reference[:3, :3].T @ current[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def _parse_guide_target(value: str | None) -> np.ndarray | None:
    if value is None:
        return None
    try:
        target = np.array([float(part.strip()) for part in value.split(",")], dtype=float)
    except ValueError as exc:
        raise SystemExit("guide-target-q must be six comma-separated joint angles in radians") from exc
    if target.shape != (6,) or not np.isfinite(target).all():
        raise SystemExit("guide-target-q must be six finite comma-separated joint angles in radians")
    return target


def _wait_for_guided_pose(robot, pose_index: int, poses: int, target: np.ndarray | None) -> None:
    if target is None:
        input(
            f"\nPose {pose_index}/{poses}: support and guide the arm to the test pose, "
            "then press Enter ONLY AFTER reaching that pose. "
            "Recording starts after the countdown and continues until support is confirmed. "
        )
        return

    print(
        f"\nPose {pose_index}/{poses}: support and hand-guide toward the target below. "
        "The target is advisory only; press Enter at a comfortable similar pose. "
        "The arm will not move automatically.",
        flush=True,
    )
    target_deg = np.degrees(target)
    while True:
        q = np.asarray(robot.get_joint_pos()[:6], dtype=float)
        error_deg = np.degrees(q - target)
        print(
            "\r"
            f"target_deg={np.array2string(target_deg, precision=1)} "
            f"current_deg={np.array2string(np.degrees(q), precision=1)} "
            f"max_error={np.max(np.abs(error_deg)):4.1f}deg   ",
            end="",
            flush=True,
        )
        ready, _, _ = select.select([sys.stdin], [], [], 0.2)
        if not ready:
            continue
        sys.stdin.readline()
        print(flush=True)
        return


def main(
    channel: str,
    output_dir: str,
    duration_s: float = 15.0,
    poses: int = 3,
    j2_gravity_factor: float | None = None,
    j3_gravity_factor: float | None = None,
    guide_target_q: str | None = None,
    keep_supported: bool = False,
    diagnostic_hold: bool = False,
    j3_bias_nm: float = 0.0,
    max_displacement_rad: float = 0.12,
    max_velocity_rad_s: float = 0.35,
    max_temperature_c: float = 60.0,
    max_torque_nm: float = 20.0,
) -> None:
    """Measure joint and model-derived tool drift at several hand-guided poses."""
    if channel not in {"can_left", "can_right"}:
        raise SystemExit("channel must be can_left or can_right")
    if not 5.0 <= duration_s <= 60.0:
        raise SystemExit("duration-s must be between 5 and 60 seconds")
    if not 1 <= poses <= 5:
        raise SystemExit("poses must be between 1 and 5")
    if j2_gravity_factor is not None and not 1.0 <= j2_gravity_factor <= 1.5:
        raise SystemExit("j2-gravity-factor must be between 1.0 and 1.5")
    if j3_gravity_factor is not None and not 1.0 <= j3_gravity_factor <= 1.5:
        raise SystemExit("j3-gravity-factor must be between 1.0 and 1.5")
    if not np.isfinite(j3_bias_nm) or not 0.0 <= j3_bias_nm <= 0.8:
        raise SystemExit("j3-bias-nm must be finite and between 0 and 0.8 Nm")
    if j3_bias_nm and (diagnostic_hold or poses != 1):
        raise SystemExit("j3-bias-nm requires one free-gravity pose, without diagnostic-hold")
    guide_target = _parse_guide_target(guide_target_q)
    if guide_target is not None and poses != 1:
        raise SystemExit("guide-target-q requires poses=1")

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    csv_path = out / "samples.csv"
    metrics_path = out / "metrics.json"

    print("Running startup checks for arm joints 1-6 and gripper ID 7...", flush=True)
    run_startup_checks(
        channel,
        ULTRA2_WITH_GRIPPER,
        check_motor_types=True,
        check_motor_config=True,
        control_mode=ControlMode.MIT,
        repair=False,
    )

    model_path = combine_arm_and_gripper_xml(ArmType.YAM_ULTRA_2, GripperType.LINEAR_4310)
    kinematics = Kinematics(model_path, "grasp_site")
    gravity_comp_factor = None
    if j2_gravity_factor is not None or j3_gravity_factor is not None:
        gravity_comp_factor = np.array([
            1.0,
            1.2 if j2_gravity_factor is None else j2_gravity_factor,
            1.1 if j3_gravity_factor is None else j3_gravity_factor,
            1.1, 1.2, 1.0,
        ])
        print(
            "Using temporary gravity_comp_factor="
            f"{np.array2string(gravity_comp_factor, precision=3)}; repository config is unchanged.",
            flush=True,
        )

    robot = get_yam_robot(
        channel=channel,
        arm_type=ArmType.YAM_ULTRA_2,
        gripper_type=GripperType.LINEAR_4310,
        gravity_comp_factor=gravity_comp_factor,
        zero_gravity_mode=False,
        gripper_limits_override=NO_CALIBRATION_LIMITS,
        gripper_kp=0.0,
        gripper_kd=0.0,
        enable_auto_recovery=False,
        use_coulomb_friction=False,
    )

    fields = ["pose", "t_s", "after_measurement_window"]
    fields += [f"q{i}_rad" for i in range(1, 7)]
    fields += [f"dq{i}_rad_s" for i in range(1, 7)]
    fields += [f"tau{i}_nm" for i in range(1, 7)]
    # tau is commanded feedforward; feedback is the motor-reported estimate,
    # not an independent measurement of mechanical output torque.
    fields += [f"feedback_tau{i}_nm" for i in range(1, 7)]
    fields += ["diagnostic_hold", "pd2_nm", "pd3_nm", "estimated_total2_nm", "estimated_total3_nm"]
    fields += ["j3_bias_nm"]
    fields += ["tool_x_m", "tool_y_m", "tool_z_m", "tool_drift_mm", "tool_rotation_deg"]
    fields += ["max_mos_c", "max_rotor_c"]
    all_metrics: list[dict[str, object]] = []

    try:
        time.sleep(1.0)
        robot.enter_gravity_comp_idle()
        print("Gravity compensation is active. Hand-guide slowly; avoid joint limits and collisions.", flush=True)
        print(f"Active gravity_comp_factor={robot.gravity_comp_factor[:6].tolist()}", flush=True)
        if keep_supported:
            print("SUPPORTED DIAGNOSTIC: keep supporting the arm throughout; do not release it.", flush=True)

        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()

            for pose_index in range(1, poses + 1):
                if diagnostic_hold and pose_index > 1:
                    # The previous pose ended only after the user confirmed support.
                    robot.enter_gravity_comp_idle()
                _wait_for_guided_pose(robot, pose_index, poses, guide_target)
                print(
                    "Keep supporting the arm and hold still during the countdown: 3...",
                    flush=True,
                )
                time.sleep(1.0)
                print("2...", flush=True)
                time.sleep(1.0)
                print("1... keep supporting", flush=True)
                time.sleep(1.0)

                hold_reference = robot.get_joint_pos().copy()
                q0 = hold_reference[:6].copy()
                if j3_bias_nm:
                    # Apply only after the supported countdown. The SDK already
                    # sums joint_commands.torques with model gravity each cycle;
                    # normal close()/zero-torque idle clears this command.
                    with robot._command_lock:
                        robot._commands.torques[2] = j3_bias_nm
                    print(
                        f"LOCAL J3 BIAS TEST: +{j3_bias_nm:.3f} Nm, position stiffness remains zero. "
                        "Stay near this pose; this is not a correction validated across the workspace.",
                        flush=True,
                    )
                if diagnostic_hold:
                    # Capture the user's current pose; never command a predefined target.
                    robot.command_joint_state({
                        "pos": hold_reference,
                        "vel": np.zeros(7),
                        "kp": DIAGNOSTIC_HOLD_KP.copy(),
                        "kd": DIAGNOSTIC_HOLD_KD.copy(),
                    })
                    print(
                        "DIAGNOSTIC POSITION HOLD active at your current pose. "
                        "Do not hand-guide while hold is active. "
                        "This measures extra holding effort; stability here is NOT a gravity-compensation pass.",
                        flush=True,
                    )
                tool0 = _tool_pose(kinematics, q0)
                started = time.monotonic()
                print(
                    "Recording now: keep supporting throughout."
                    if keep_supported else
                    "Recording now: gradually reduce support; catch immediately if it drifts.",
                    flush=True,
                )
                samples: list[dict[str, float | int]] = []
                next_report = started
                awaiting_support = False

                while True:
                    now = time.monotonic()
                    elapsed = now - started
                    if elapsed >= duration_s and not awaiting_support:
                        awaiting_support = True
                        print(
                            "Measurement window ended; the current control mode, recording and guards remain active. "
                            "Support the arm NOW without repositioning it, then press Enter to finish this pose.",
                            flush=True,
                        )

                    states = robot.motor_chain.read_states()
                    observations = robot.get_observations()
                    q = np.asarray(observations["joint_pos"][:6], dtype=float).copy()
                    dq = np.asarray(observations["joint_vel"][:6], dtype=float).copy()
                    feedback_tau = np.asarray(observations["joint_eff"][:6], dtype=float).copy()
                    torques = robot.get_motor_torques()
                    tau = np.asarray(torques[:6], dtype=float) if torques is not None else np.zeros(6)
                    # J2/J3 are direct joint coordinates on Ultra2. This estimate
                    # combines the last feedforward with the present PD error;
                    # it is not a synchronized or independent torque measurement.
                    if diagnostic_hold:
                        pd23 = DIAGNOSTIC_HOLD_KP[1:3] * (q0[1:3] - q[1:3]) - DIAGNOSTIC_HOLD_KD[1:3] * dq[1:3]
                    else:
                        pd23 = -robot._grav_comp_kd[1:3] * dq[1:3]
                    total23 = tau[1:3] + pd23
                    tool = _tool_pose(kinematics, q)
                    tool_drift_mm = float(np.linalg.norm(tool[:3, 3] - tool0[:3, 3]) * 1000.0)
                    tool_rotation_deg = _rotation_error_deg(tool0, tool)
                    mos = np.array([state.temp_mos for state in states], dtype=float)
                    rotor = np.array([state.temp_rotor for state in states], dtype=float)

                    max_joint_drift = float(np.max(np.abs(q - q0)))
                    max_velocity = float(np.max(np.abs(dq)))
                    max_torque = float(max(np.max(np.abs(tau)), np.max(np.abs(feedback_tau)), np.max(np.abs(total23))))
                    max_temperature = float(max(np.max(mos), np.max(rotor)))
                    bad_errors = [
                        (index + 1, state.error_code)
                        for index, state in enumerate(states)
                        if state.error_code != "0x1"
                    ]
                    if bad_errors:
                        raise RuntimeError(f"motor error(s): {bad_errors}")
                    if max_joint_drift > max_displacement_rad:
                        raise RuntimeError(
                            f"drift guard: {max_joint_drift:.5f} rad > {max_displacement_rad:.5f} rad"
                        )
                    if max_velocity > max_velocity_rad_s:
                        raise RuntimeError(
                            f"velocity guard: {max_velocity:.5f} rad/s > {max_velocity_rad_s:.5f} rad/s"
                        )
                    if max_temperature >= max_temperature_c:
                        raise RuntimeError(
                            f"temperature guard: {max_temperature:.1f} C >= {max_temperature_c:.1f} C"
                        )
                    if max_torque > max_torque_nm:
                        raise RuntimeError(f"torque guard: {max_torque:.3f} Nm > {max_torque_nm:.3f} Nm")

                    row: dict[str, float | int] = {
                        "pose": pose_index,
                        "t_s": elapsed,
                        "after_measurement_window": int(awaiting_support),
                    }
                    row.update({f"q{i + 1}_rad": float(q[i]) for i in range(6)})
                    row.update({f"dq{i + 1}_rad_s": float(dq[i]) for i in range(6)})
                    row.update({f"tau{i + 1}_nm": float(tau[i]) for i in range(6)})
                    row.update({f"feedback_tau{i + 1}_nm": float(feedback_tau[i]) for i in range(6)})
                    row.update({
                        "diagnostic_hold": int(diagnostic_hold),
                        "pd2_nm": float(pd23[0]), "pd3_nm": float(pd23[1]),
                        "estimated_total2_nm": float(total23[0]), "estimated_total3_nm": float(total23[1]),
                        "j3_bias_nm": j3_bias_nm,
                    })
                    row.update(
                        {
                            "tool_x_m": float(tool[0, 3]),
                            "tool_y_m": float(tool[1, 3]),
                            "tool_z_m": float(tool[2, 3]),
                            "tool_drift_mm": tool_drift_mm,
                            "tool_rotation_deg": tool_rotation_deg,
                            "max_mos_c": float(np.max(mos)),
                            "max_rotor_c": float(np.max(rotor)),
                        }
                    )
                    writer.writerow(row)
                    samples.append(row)

                    if now >= next_report:
                        print(
                            f"pose={pose_index} t={elapsed:5.1f}s "
                            f"tool_drift={tool_drift_mm:6.2f}mm rotation={tool_rotation_deg:5.2f}deg "
                            f"max_joint_drift={np.degrees(max_joint_drift):5.2f}deg "
                            f"J2={np.degrees(q[1]):+.1f}deg "
                            f"cmd/fb={tau[1]:+.3f}/{feedback_tau[1]:+.3f}Nm "
                            f"J3={np.degrees(q[2]):+.1f}deg "
                            f"cmd/fb={tau[2]:+.3f}/{feedback_tau[2]:+.3f}Nm",
                            flush=True,
                        )
                        if diagnostic_hold:
                            print(
                                f"  additional PD J2/J3={pd23[0]:+.3f}/{pd23[1]:+.3f}Nm; "
                                "keep hands close; report whether support was fully removed.",
                                flush=True,
                            )
                        handle.flush()
                        next_report += 1.0
                    if awaiting_support:
                        ready, _, _ = select.select([sys.stdin], [], [], 0)
                        if ready:
                            if sys.stdin.readline() == "":
                                raise RuntimeError("stdin closed before physical support was confirmed")
                            break
                    time.sleep(0.02)

                q_samples = np.array([[float(row[f"q{i}_rad"]) for i in range(1, 7)] for row in samples])
                final_q = q_samples[-1]
                final_tool = np.array(
                    [
                        float(samples[-1]["tool_x_m"]),
                        float(samples[-1]["tool_y_m"]),
                        float(samples[-1]["tool_z_m"]),
                    ]
                )
                pose_metrics: dict[str, object] = {
                    "pose": pose_index,
                    "duration_s": duration_s,
                    "recorded_duration_s": float(samples[-1]["t_s"]),
                    "includes_support_confirmation_phase": True,
                    "keep_supported": keep_supported,
                    "diagnostic_hold": diagnostic_hold,
                    "j3_bias_nm": j3_bias_nm,
                    "hold_kp": DIAGNOSTIC_HOLD_KP.tolist() if diagnostic_hold else None,
                    "hold_kd": DIAGNOSTIC_HOLD_KD.tolist() if diagnostic_hold else None,
                    "gravity_comp_factor": robot.gravity_comp_factor[:6].tolist(),
                    "reference_q_rad": q0.tolist(),
                    "final_joint_drift_deg": np.degrees(final_q - q0).tolist(),
                    "peak_joint_drift_deg": np.degrees(np.max(np.abs(q_samples - q0), axis=0)).tolist(),
                    "max_tool_drift_mm": max(float(row["tool_drift_mm"]) for row in samples),
                    "final_tool_drift_mm": float(np.linalg.norm(final_tool - tool0[:3, 3]) * 1000.0),
                    "max_tool_rotation_deg": max(float(row["tool_rotation_deg"]) for row in samples),
                    "max_velocity_rad_s": max(
                        max(abs(float(row[f"dq{i}_rad_s"])) for i in range(1, 7)) for row in samples
                    ),
                    "max_torque_nm": max(
                        max(abs(float(row[f"tau{i}_nm"])) for i in range(1, 7)) for row in samples
                    ),
                    "max_feedback_torque_nm": max(
                        max(abs(float(row[f"feedback_tau{i}_nm"])) for i in range(1, 7)) for row in samples
                    ),
                    "max_temperature_c": max(
                        max(float(row["max_mos_c"]), float(row["max_rotor_c"])) for row in samples
                    ),
                }
                if guide_target is not None:
                    pose_metrics["guide_target_q_rad"] = guide_target.tolist()
                    pose_metrics["guide_target_max_error_deg"] = float(
                        np.max(np.abs(np.degrees(q0 - guide_target)))
                    )
                pose_metrics["initial_acceptance"] = bool(
                    float(pose_metrics["max_tool_drift_mm"]) <= 20.0
                    and max(float(v) for v in pose_metrics["peak_joint_drift_deg"]) <= 2.0
                )
                if keep_supported or diagnostic_hold:
                    # Neither hand support nor PD hold establishes free-floating stability.
                    pose_metrics["initial_acceptance"] = None
                all_metrics.append(pose_metrics)
                metrics_path.write_text(json.dumps(all_metrics, indent=2), encoding="utf-8")
                print(
                    f"Pose {pose_index} complete: max tool drift={pose_metrics['max_tool_drift_mm']:.2f} mm, "
                    f"max joint drift={max(pose_metrics['peak_joint_drift_deg']):.2f} deg, "
                    f"initial_acceptance={pose_metrics['initial_acceptance']} "
                    "(recorded drift thresholds only; hand support and field observations determine validity)",
                    flush=True,
                )

        print("\nAll poses recorded and support confirmed. Keep supporting while motors close.", flush=True)
        print(f"Metrics: {metrics_path}", flush=True)
        print(f"Samples: {csv_path}", flush=True)
    finally:
        _close_robot(robot)


if __name__ == "__main__":
    tyro.cli(main)
