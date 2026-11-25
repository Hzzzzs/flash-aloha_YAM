"""Guarded simultaneous gravity-compensation test for two YAM Ultra2 arms."""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import tyro

from i2rt.robots.get_robot import get_yam_robots
from i2rt.robots.utils import ArmType, GripperType


NO_CALIBRATION_LIMITS = np.array([-3.3, 3.3])


def main(
    output_dir: str,
    duration_s: float = 3.0,
    max_displacement_rad: float = 0.08,
    max_velocity_rad_s: float = 0.35,
    max_temperature_c: float = 60.0,
    max_torque_nm: float = 20.0,
) -> None:
    if duration_s not in {3.0, 10.0}:
        raise SystemExit("duration-s must be 3 for the first stage or 10 after review")

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    robots = []
    labels = ["left", "right"]
    metrics: dict[str, dict[str, float | list[float]]] = {}

    try:
        print("Creating left and right yam_ultra_2 + linear_4310 controllers...", flush=True)
        robots = get_yam_robots(
            arms=[{"channel": "can_left"}, {"channel": "can_right"}],
            arm_type=ArmType.YAM_ULTRA_2,
            gripper_type=GripperType.LINEAR_4310,
            zero_gravity_mode=False,
            gripper_limits_override=NO_CALIBRATION_LIMITS,
            gripper_kp=0.0,
            gripper_kd=0.0,
            enable_auto_recovery=False,
            use_coulomb_friction=False,
        )
        time.sleep(1.0)

        for robot in robots:
            states = robot.motor_chain.read_states()
            bad = [(i + 1, state.error_code) for i, state in enumerate(states) if state.error_code != "0x1"]
            if bad:
                raise RuntimeError(f"motor error before dual gravity compensation: {bad}")
            robot.enter_gravity_comp_idle()

        print("Both arms are now in gravity compensation.", flush=True)
        answer = input(
            "Gradually unload LEFT, then RIGHT, while keeping supports directly below both. "
            "Type BOTH_FLOATING only after both are stable: "
        )
        if answer != "BOTH_FLOATING":
            raise RuntimeError("dual-arm floating confirmation not received")

        q0 = {label: robot.get_joint_pos()[:6].copy() for label, robot in zip(labels, robots, strict=True)}
        maxima = {
            label: {"displacement": 0.0, "velocity": 0.0, "torque": 0.0, "temperature": 0.0}
            for label in labels
        }
        started = time.monotonic()
        next_report = started

        while True:
            now = time.monotonic()
            elapsed = now - started
            if elapsed >= duration_s:
                break

            reports = []
            for label, robot in zip(labels, robots, strict=True):
                states = robot.motor_chain.read_states()
                obs = robot.get_observations()
                q = np.asarray(obs["joint_pos"][:6], dtype=float).copy()
                dq = np.asarray(obs["joint_vel"][:6], dtype=float).copy()
                torques = robot.get_motor_torques()
                tau = np.asarray(torques[:6], dtype=float) if torques is not None else np.zeros(6)
                mos = np.array([state.temp_mos for state in states], dtype=float)
                rotor = np.array([state.temp_rotor for state in states], dtype=float)

                displacement = float(np.max(np.abs(q - q0[label])))
                velocity = float(np.max(np.abs(dq)))
                torque = float(np.max(np.abs(tau)))
                temperature = float(max(np.max(mos), np.max(rotor)))
                maxima[label]["displacement"] = max(maxima[label]["displacement"], displacement)
                maxima[label]["velocity"] = max(maxima[label]["velocity"], velocity)
                maxima[label]["torque"] = max(maxima[label]["torque"], torque)
                maxima[label]["temperature"] = max(maxima[label]["temperature"], temperature)

                bad = [(i + 1, state.error_code) for i, state in enumerate(states) if state.error_code != "0x1"]
                if bad:
                    raise RuntimeError(f"{label} motor error(s): {bad}")
                if displacement > max_displacement_rad:
                    raise RuntimeError(f"{label} drift guard: {displacement:.5f} rad")
                if velocity > max_velocity_rad_s:
                    raise RuntimeError(f"{label} velocity guard: {velocity:.5f} rad/s")
                if temperature >= max_temperature_c:
                    raise RuntimeError(f"{label} temperature guard: {temperature:.1f} C")
                if torque > max_torque_nm:
                    raise RuntimeError(f"{label} torque guard: {torque:.3f} Nm")
                reports.append(
                    f"{label}:drift={np.degrees(displacement):.3f}deg "
                    f"vel={velocity:.3f}rad/s tau={torque:.2f}Nm temp={temperature:.0f}C"
                )

            if now >= next_report:
                print(f"t={elapsed:.2f}s " + " | ".join(reports), flush=True)
                next_report += 0.5
            time.sleep(0.02)

        for label in labels:
            values = maxima[label]
            metrics[label] = {
                "duration_s": duration_s,
                "reference_q_rad": q0[label].tolist(),
                "max_displacement_rad": values["displacement"],
                "max_displacement_deg": float(np.degrees(values["displacement"])),
                "max_velocity_rad_s": values["velocity"],
                "max_torque_nm": values["torque"],
                "max_temperature_c": values["temperature"],
            }
        (out / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")

        print("PASS simultaneous dual-arm gravity compensation", flush=True)
        for label in labels:
            values = metrics[label]
            print(
                f"{label}: max_drift={values['max_displacement_deg']:.3f}deg "
                f"max_velocity={values['max_velocity_rad_s']:.3f}rad/s "
                f"max_torque={values['max_torque_nm']:.3f}Nm "
                f"max_temperature={values['max_temperature_c']:.0f}C",
                flush=True,
            )
        answer = input("Place BOTH arms back on their supports, then type BOTH_SUPPORTED to close: ")
        if answer != "BOTH_SUPPORTED":
            raise RuntimeError("support confirmation not received")
    finally:
        print("Closing both controllers; both arms will become limp.", flush=True)
        for robot in robots:
            try:
                robot.close()
            except Exception as exc:
                print(f"ERROR while closing robot: {exc!r}", flush=True)


if __name__ == "__main__":
    tyro.cli(main)
