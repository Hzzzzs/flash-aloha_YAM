"""Operator-run YAM dual-arm RGB collection using Flash's native stream contracts.

Camera-only mode exercises the same encoder, frame references and disk path
without constructing a robot. Actions in hand-guided mode are same-row measured
absolute poses, not motor torque commands. No old trajectory is loaded.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime
from dataclasses import asdict
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import numpy as np

from config.paths import DATA_DIR, LOG_DIR
from camera_utils.camera_timing import TimingLimits
from contracts.standard_capture import capture_observation, empty_robot_state
from hardware.safe_exit import damping_exit, raising_signals
from recording.practice_support import RunContext, TerminalKeyWatcher
from recording.recording_utils import atomic_write_json, summarize_recording_rate

NAMES = ['video_left', 'video_right']
FPS = 30.0
WIDTH, HEIGHT = 640, 480
DEVICES = {role: os.environ.get(f'YAM_CAMERA_{role.upper()}',
                               f'/dev/v4l/by-path/pci-0000:00:14.0-usb-0:{port}:1.0-video-index4')
           for role, port in [('left', '5.3'), ('right', '5.4')]}
GRIPPER_KEYS = {'a': ('left', 0.), 's': ('left', 1.),
                'k': ('right', 0.), 'l': ('right', 1.)}
ROBOT_CONTRACT = {'robot_contract_version': 1,
                  'arms': {role: {'model': 'yam_ultra_2'} for role in ('left', 'right')}}


def recording_key(keys, backend, *, pick_place=False):
    """Handle one operator key before the standard observation boundary."""
    key = keys.poll()
    if key == '1':
        return True
    if pick_place and key in GRIPPER_KEYS:
        role, opening = GRIPPER_KEYS[key]
        backend.command_teaching_gripper(role, opening)
        print(f'[Gripper command] {role}: {"OPEN" if opening else "CLOSE"} '
              '(position request; grasp success must be checked by operator)', flush=True)
    return False


def record_samples(read_state, recorder, *, duration, should_stop=lambda: False,
                   clock=time.monotonic, sleep=time.sleep, capture=capture_observation):
    """Absolute deadlines; store precisely the selected observation and frame IDs."""
    stamps = []
    deadline = clock()
    for _ in range(round(duration * FPS)):
        sleep(max(0., deadline - clock()))
        if should_stop():
            break
        observation = capture(read_state, NAMES)
        recorder.append_sample({**observation.recording_fields(),
                                'commanded_action': observation.qpos.copy()})
        stamps.append(observation.robot_timestamp)
        # Never burst to manufacture missing samples after an overrun.
        deadline = max(deadline + 1 / FPS, clock())
    report = summarize_recording_rate(FPS, np.diff(stamps), sample_count=len(stamps))
    report['p95_interval_ms'] = float(np.percentile(np.diff(stamps), 95) * 1000) if len(stamps)>1 else None
    return report


def stop_camera(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def wait_camera_startup(processes, *, directory='/dev/shm', timeout=20):
    """Wait for producer declarations before starting fail-closed consumers.

    SharedMemory becomes visible before its inode-bound clock declaration is
    published. Waiting here is startup orchestration, never a live-frame retry.
    """
    from camera_utils.camera_timing import CameraTimingError, validate_shared_clock_contract
    pending = dict(processes)
    deadline = time.monotonic() + timeout
    while pending:
        for role, process in processes.items():
            if process.poll() is not None:
                raise RuntimeError(f'{role} camera exited ({process.returncode}); see camera-{role}.log')
        for role in list(pending):
            path = Path(directory) / f'flash_aloha_cam_{role}'
            try:
                with path.open('rb') as source:
                    validate_shared_clock_contract(
                        SimpleNamespace(_fd=source.fileno(), name=path.name), directory=directory)
            except (FileNotFoundError, CameraTimingError) as exc:
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'{role} camera startup timed out: {exc}') from exc
            else:
                del pending[role]
        if pending:
            time.sleep(.01)


def start_cameras(stack, log_dir):
    """Only launch our two camera processes; never replace an existing server."""
    import socket
    for port in (55551, 55553):
        with socket.socket() as sock:
            if sock.connect_ex(('127.0.0.1', port)) == 0:
                raise RuntimeError(f'Camera port {port} already in use; stop its owner first')
    for role, device in DEVICES.items():
        if not Path(device).exists():
            raise FileNotFoundError(f'{role} D405 is missing: {device}')
    processes = {}
    for role, device in DEVICES.items():
        log = stack.enter_context((log_dir / f'camera-{role}.log').open('w'))
        process = subprocess.Popen([sys.executable, '-u', str(ROOT / 'src/camera_utils/camera_server.py'),
            '--camera', role, '--device', device, '--width', str(WIDTH), '--height', str(HEIGHT),
            '--fps', str(FPS), '--no-default-controls', '--write-shared-memory',
            '--robologger-codec', 'libx264'], stdout=log, stderr=subprocess.STDOUT)
        stack.callback(stop_camera, process)
        processes[role] = process
    wait_camera_startup(processes)


def wait_key(keys, allowed, backend=None):
    while True:
        if backend is not None:
            backend.passive_step()
        key = keys.poll()
        if key in allowed:
            return key
        time.sleep(.02)


def review_recording(backend, manifest, keys):
    """Review this stopped raw take through the native replay; never append rows."""
    from recording.recording_replay import load_replay_episode, replay_take

    while True:
        print('Take is on disk; arms are holding. 1=save, h=discard, r=prepare replay.\n'
              'r actively moves BOTH arms and grippers to the recorded first pose. Clear the path.\n'
              'At the first pose: 1=play, h=cancel. During replay: 1=stop into damping.\n'
              'Save/discard returns arms to startup poses, then disables: support them.', flush=True)
        choice = wait_key(keys, {'1', 'h', 'r'}, backend)
        if choice != 'r':
            return choice == '1'
        record = json.loads(Path(manifest).read_text())
        if not all(record.get(field) is True for field in ('complete', 'retained', 'alignable')):
            raise ValueError('Replay requires a complete, retained, alignable take')
        actions, camera, reference = load_replay_episode(
            Path(manifest).parent, camera_names=NAMES,
            robot_contract=backend.ctx.robot_contract, fps=backend.ctx.fps)
        # Validate every target before the native replay moves to its first row.
        if actions.shape[1:] != (14,) or not np.isfinite(actions).all():
            raise ValueError('Dual YAM replay requires 14 finite positions per row')
        if np.any((actions[:, [6, 13]] < 0) | (actions[:, [6, 13]] > 1)):
            raise ValueError('Replay gripper positions must be in [0, 1]')

        def live_frame(name):
            return capture_observation(backend.read_state, NAMES, include_images=True).images[name]

        live_frame(camera)  # Check the live camera contract before starting motion.
        result = replay_take(backend, actions, camera, reference,
            label=str(Path(manifest).parent), gain_scales=(1.0, 1.0),
            poll_keys=lambda: [key] if (key := keys.poll()) else [], live_frame=live_frame)
        print(f'Replay result: {result}. Original take is unchanged.', flush=True)
        if result == 'quit':
            print('Replay exited. Choose 1 to save the take or h to discard.', flush=True)
            return wait_key(keys, {'1', 'h'}, backend) == '1'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-only', action='store_true', help='No robot connection; record both RGB streams')
    parser.add_argument('--pick-place', action='store_true',
                        help='Hand-guide arms; a/s close/open LEFT, k/l close/open RIGHT during recording')
    parser.add_argument('--duration', type=int, default=10, choices=range(5, 61), metavar='5..60')
    args = parser.parse_args()
    if args.camera_only and args.pick_place:
        parser.error('--pick-place requires the robot; cannot combine with --camera-only')
    # Import storage/transport before touching cameras or motors.
    from recording.distributed_recording import DistributedRecorder, EpisodeRequest, publish_recording
    from camera_utils.capture import acquire_camera_readers
    from robologger.loggers.video_logger import VideoLogger  # noqa: F401

    tag = datetime.now().strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:6]
    log_dir = LOG_DIR / 'yam_capture' / tag
    log_dir.mkdir(parents=True)
    dataset = DATA_DIR / ('yam_camera_check_30hz' if args.camera_only else 'yam_bimanual_30hz')
    dataset.mkdir(parents=True, exist_ok=True)
    params = ROOT / 'calibration/d405_640x480_30.npz'
    os.environ['FLASH_ALOHA_CAMERA_PARAMS'] = str(params)
    timing_limits = TimingLimits.from_env()
    print(f'30 Hz collection; two RGB cameras at {WIDTH}x{HEIGHT}@{FPS:g}; logs: {log_dir}', flush=True)
    print('Camera timing limits (ms): ' + ', '.join(
        f'{key.removesuffix("_seconds")}={value * 1000:g}'
        for key, value in asdict(timing_limits).items()), flush=True)
    if not args.camera_only:
        print('Both arms ON and supported; clear grippers for startup calibration. No old trajectory is loaded.')
        if input('Type YAM_CAPTURE_READY to connect: ').strip() != 'YAM_CAPTURE_READY':
            return
    backend = None
    manifest = None
    report = None
    saved = False
    with ExitStack() as cleanup:
        start_cameras(cleanup, log_dir)
        lease = acquire_camera_readers(NAMES, timeout=20)
        cleanup.callback(lease.close)
        # Confirm actual frames, not just requested camera settings, before CAN startup.
        preview = capture_observation(empty_robot_state, NAMES, include_images=True)
        import cv2
        for name, frame in preview.images.items():
            if frame.shape != (HEIGHT, WIDTH, 3):
                raise RuntimeError(f'{name}: unexpected frame shape {frame.shape}')
            cv2.imwrite(str(log_dir / f'{name}.jpg'), frame)
        recorder = DistributedRecorder(camera_names=NAMES, data_dir=str(dataset / 'data'))
        cleanup.callback(recorder.close)
        with TerminalKeyWatcher() as keys:
            exit_state = None
            try:
                with raising_signals(), damping_exit(
                        lambda: backend.damping_target() if backend else None,
                        home_requested=keys.home_requested) as exit_state:
                    if not args.camera_only:
                        from hardware.backends.yam import YAMBackend
                        ctx = RunContext('yam_bimanual_30hz', str(dataset), args.duration * 30,
                            NAMES, 'yam', False, True, can_interface_left='can_left',
                            can_interface_right='can_right', fps=FPS, robot_contract=ROBOT_CONTRACT)
                        backend = YAMBackend(ctx)
                        backend.setup_for_recording()
                        backend.enable_passive_compensation()
                        print('Guide both arms to a comfortable pose. Press 1 to record, q to exit.', flush=True)
                        if wait_key(keys, {'1', 'q'}, backend) == 'q':
                            backend.shutdown(home=True)
                            return
                    metadata = dict(robot_type='camera_only' if args.camera_only else 'yam',
                        robot_model='none' if args.camera_only else 'yam_ultra_2',
                        fps=FPS, control_freq=FPS, nominal_recording_hz=FPS, action_rate_hz=FPS,
                        action_command_style='absolute', action_label_source='same_row_measured_qpos',
                        teaching_gripper_control='keyboard_position' if args.pick_place else 'manual',
                        gripper_force_limit_n=5.0 if args.pick_place else None,
                        task_name='bimanual_pick_place' if args.pick_place else 'hand_guided_check',
                        gripper_units='normalized_0_closed_1_open',
                        camera_timing_limits=asdict(timing_limits),
                        camera_mode=[WIDTH, HEIGHT, FPS], camera_names=NAMES)
                    if backend:
                        metadata['robot_contract'] = ROBOT_CONTRACT
                    recorder.start_episode(EpisodeRequest(
                        session_id=tag, data_dir=str(dataset / 'data'), camera_names=NAMES,
                        record_rollout=not args.camera_only, record_dagger=False,
                        rollout_dir=str(dataset), rollout_task_name='', dagger_dir='', dagger_task_name='',
                        dagger_camera_names=[], is_double_arm=not args.camera_only, fps=FPS, metadata=metadata), timeout=20)
                    complete = False
                    try:
                        print(f'Recording {args.duration}s. Move slowly; 1 stops early.', flush=True)
                        if args.pick_place:
                            print('Gripper keys: a=LEFT CLOSE, s=LEFT OPEN; k=RIGHT CLOSE, l=RIGHT OPEN.\n'
                                  'Clear fingers before pressing. Place and release both objects before time ends.\n'
                                  'Arms stay hand-guided; grippers use the configured 5 N force limit.', flush=True)
                        report = record_samples(backend.read_state if backend else empty_robot_state, recorder,
                            duration=args.duration, should_stop=lambda: recording_key(
                                keys, backend, pick_place=args.pick_place))
                        if backend:
                            backend.hold_after_recording()
                        complete = True
                    finally:
                        manifest = recorder.stop_episode(retained=complete,
                            outputs={'rollout': {'requested': not args.camera_only, 'decision': 'success' if complete else 'discard'}},
                            incomplete=not complete, timeout=30)
                    atomic_write_json(log_dir / 'rate.json', report)
                    print(json.dumps(report, indent=2), flush=True)
                    if backend:
                        saved = review_recording(backend, manifest, keys)
                        if not saved:
                            record = json.loads(Path(manifest).read_text())
                            record['retained'] = False
                            record['outputs']['rollout']['decision'] = 'discard'
                            atomic_write_json(manifest, record)
                        backend.shutdown(home=True)
            finally:
                if backend:
                    backend.shutdown(home=False)
    # Native publication after all robot and camera operations have stopped.
    if saved and manifest:
        print('Published:', publish_recording(manifest))
    print('Raw manifest:', manifest)
    print('Logs:', log_dir)


if __name__ == '__main__':
    main()
