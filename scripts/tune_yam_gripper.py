"""Tune manual YAM Ultra 2 / linear_4310 gripper assistance, one arm at a time.

Arm joints hold their measured starting pose; only the gripper is hand-guided.
Startup calibrates the gripper. Assistance starts OFF, even with a saved preset.
"""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from config.paths import LOG_DIR
from contracts import capture_gc
from hardware.backends.yam import create_yam_controllers, YAMDamping
from hardware.safe_exit import damping_exit, raising_signals
from hardware.yam_gripper_friction import GripperFriction, load_config, save_config
from recording.practice_support import TerminalKeyWatcher


def adjust(parameters, key):
    if key == 'a':
        return replace(parameters, acceleration_enabled=not parameters.acceleration_enabled)
    changes = {'+': ('coulomb_nm', .005, 0., parameters.max_torque_nm),
               '=': ('coulomb_nm', .005, 0., parameters.max_torque_nm),
               '-': ('coulomb_nm', -.005, 0., parameters.max_torque_nm),
               ']': ('viscous_nm_s_rad', .001, 0., .02),
               '[': ('viscous_nm_s_rad', -.001, 0., .02),
               '.': ('transition_rad_s', .05, .02, 2.),
               ',': ('transition_rad_s', -.05, .02, 2.),
               'u': ('acceleration_gain_nm_s2_rad', .0001, 0., .002),
               'j': ('acceleration_gain_nm_s2_rad', -.0001, 0., .002)}
    if key not in changes:
        return parameters
    name, delta, low, high = changes[key]
    return replace(parameters, **{name: round(max(low, min(high, getattr(parameters, name) + delta)), 6)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', choices=['left', 'right'], required=True)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/yam-gripper-friction.json')
    args = parser.parse_args()
    parameters = (load_config(args.config).get(args.arm, GripperFriction())
                  if args.config.exists() else GripperFriction())
    print(f'Only {args.arm} arm (can_{args.arm}) will connect. Stop other CAN controllers first.')
    print('Support the arm; clear the gripper for startup calibration. Then arm joints HOLD the current pose.')
    print('Manual gripper only. Assistance starts OFF. q closes motors in place: support the arm BEFORE q.')
    if input('Type YAM_GRIPPER_READY to connect: ').strip() != 'YAM_GRIPPER_READY':
        return
    controllers = {}
    enabled = False
    directory = LOG_DIR / 'yam_gripper_tuning'
    directory.mkdir(parents=True, exist_ok=True)
    log_path = directory / f'{time.strftime("%Y%m%d-%H%M%S")}-{args.arm}.jsonl'
    try:
        with TerminalKeyWatcher() as keys, log_path.open('w') as log:
            with raising_signals(), damping_exit(
                    lambda: YAMDamping(list(controllers.values())) if controllers else None,
                    home_requested=keys.home_requested):
                capture_gc.pause_for_stream()
                controllers = create_yam_controllers({args.arm: f'can_{args.arm}'})
                controller = controllers[args.arm]
                controller.configure_gripper_friction(parameters, enabled=False)
                controller.enter_gripper_tuning()
                print('1=master ON/OFF; +/-=Coulomb 0.005 Nm; [/]=viscous; ,/.=transition speed; '
                      'a=acceleration ON/OFF; u/j=acceleration gain +/-0.0001; s=save; q=exit')
                print('Release the handle briefly between trials: unwanted self-motion means press 1 OFF and reduce compensation.')
                next_status = 0.
                while True:
                    controller.check()  # Also renews the 0.5 s assist heartbeat.
                    key = keys.poll()
                    if key == 'q':
                        break
                    changed = False
                    if key == '1':
                        enabled, changed = not enabled, True
                    elif key == 's':
                        save_config(args.config, args.arm, parameters)
                        print(f'Saved {args.arm} parameters to {args.config}. Collection requires explicit --gripper-friction-config.')
                    else:
                        updated = adjust(parameters, key)
                        changed = updated != parameters
                        parameters = updated
                    if changed:
                        controller.configure_gripper_friction(parameters, enabled=enabled)
                    if changed or time.monotonic() >= next_status:
                        # Only settings/events, not a new observation/trajectory recorder.
                        event = dict(time=time.time(), arm=args.arm, enabled=enabled, parameters=asdict(parameters))
                        log.write(json.dumps(event) + '\n')
                        log.flush()
                        print(f'{"ON" if enabled else "OFF"} Fc={parameters.coulomb_nm:.3f} Nm '
                              f'B={parameters.viscous_nm_s_rad:.3f} Nm*s/rad '
                              f'v0={parameters.transition_rad_s:.2f} rad/s '
                              f'accel={"ON" if parameters.acceleration_enabled else "OFF"} '
                              f'Ka={parameters.acceleration_gain_nm_s2_rad:.4f} Nm*s^2/rad '
                              f'assist={controller._assist_torque:+.4f} Nm (command, not measured force)')
                        next_status = time.monotonic() + 1.
                    time.sleep(.02)
    finally:
        for controller in controllers.values():
            controller.close()
    print(f'Tuning settings log: {log_path}')


if __name__ == '__main__':
    main()
