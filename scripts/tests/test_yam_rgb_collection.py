"""Synthetic camera/recording checks; never opens CAN or video devices."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from camera_utils.camera_server import V4L2MmapCamera, _fourcc, resolve_camera_config
from contracts.standard_capture import capture_observation

spec = importlib.util.spec_from_file_location('record_yam', Path(__file__).parents[1] / 'record_yam.py')
collector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(collector)


def test_camera_paths_can_be_mapped_without_changing_capture(monkeypatch):
    monkeypatch.setenv('YAM_CAMERA_LEFT', '/dev/v4l/by-id/left-color')
    monkeypatch.setenv('YAM_CAMERA_RIGHT', '/dev/v4l/by-id/right-color')
    mapped = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mapped)
    assert mapped.DEVICES == {'left': '/dev/v4l/by-id/left-color',
                              'right': '/dev/v4l/by-id/right-color'}
    assert (mapped.FPS, mapped.WIDTH, mapped.HEIGHT) == (30., 640, 480)


@pytest.mark.parametrize('mode', ['--camera-only', '--pick-place'])
def test_friction_assist_rejects_nonmanual_modes_before_loading_config(monkeypatch, mode):
    monkeypatch.setattr('sys.argv', ['record_yam.py', mode,
                                    '--gripper-friction-config', '/does-not-exist.json'])
    with pytest.raises(SystemExit) as result:
        collector.main()
    assert result.value.code == 2


def test_pick_place_keys_do_not_change_default_manual_teaching():
    commands = []
    backend = SimpleNamespace(command_teaching_gripper=lambda *args: commands.append(args))
    for key in ('a', 's', 'k', 'l'):
        assert not collector.recording_key(SimpleNamespace(poll=lambda: key), backend)
    assert not commands
    for key in ('a', 's', 'k', 'l', 'x', None):
        assert not collector.recording_key(SimpleNamespace(poll=lambda: key), backend, pick_place=True)
    assert commands == [('left', 0.), ('left', 1.), ('right', 0.), ('right', 1.)]
    assert collector.recording_key(SimpleNamespace(poll=lambda: '1'), backend, pick_place=True)
    assert len(commands) == 4


@pytest.mark.parametrize('stale', [False, True])
def test_startup_waits_for_clock_declaration_before_starting_readers(tmp_path, monkeypatch, stale):
    from camera_utils.camera_timing import CameraTimingError, publish_shared_clock_contract
    from camera_utils.robotmq_camera_cache import CameraSpec, SourceReader
    processes = {role: SimpleNamespace(poll=lambda: None) for role in ('left', 'right')}
    paths = [tmp_path / f'flash_aloha_cam_{role}' for role in processes]
    for path in paths:
        path.write_bytes(bytes(28))
        if stale:
            path.with_suffix('.clock.json').write_text(json.dumps(dict(
                schema_version=1, timestamp_source='v4l2_monotonic', device=-1, inode=-1, size=28)))
    # Reproduce the exact consumer failure while the producer is still starting.
    reader = SourceReader(CameraSpec('video_left', str(paths[0]), 2, 2))
    try:
        with pytest.raises(CameraTimingError, match='clock contract'):
            reader.read()
        waits = []
        def publish_after_wait(delay):
            waits.append(delay)
            for path in paths:
                with path.open('rb') as stream:
                    publish_shared_clock_contract(SimpleNamespace(_fd=stream.fileno(), name=path.name),
                                                  directory=tmp_path)
        monkeypatch.setattr(collector.time, 'sleep', publish_after_wait)
        collector.wait_camera_startup(processes, directory=tmp_path)
        assert waits == [.01]
        assert reader.read() is None  # valid clock; producer has not sent a frame yet
    finally:
        reader.close()


def test_startup_keeps_timeout_and_process_failure(tmp_path):
    with pytest.raises(RuntimeError, match='startup timed out'):
        collector.wait_camera_startup({'left': SimpleNamespace(poll=lambda: None)},
                                      directory=tmp_path, timeout=0)
    with pytest.raises(RuntimeError, match=r'left camera exited \(2\)'):
        collector.wait_camera_startup({'left': SimpleNamespace(poll=lambda: 2, returncode=2)},
                                      directory=tmp_path)


def test_yuyv_decode_preserves_padding_and_color():
    cam = V4L2MmapCamera('/not/a/device', 2, 2, 30)
    cam.pixel_format = _fourcc('YUYV')
    cam.bytesperline = 8
    pixels = np.array([[[80, 100], [120, 180]], [[90, 140], [130, 110]]], dtype=np.uint8)
    padded = np.zeros((2, 8), dtype=np.uint8)
    padded[:, :4] = pixels.reshape(2, 4)
    np.testing.assert_array_equal(cam._decode_frame(padded.tobytes()),
                                  cv2.cvtColor(pixels, cv2.COLOR_YUV2BGR_YUY2))
    assert cam._decode_frame(b'bad') is None
    cam.bytesperline = 2
    with pytest.raises(RuntimeError, match='stride'):
        cam._decode_frame(padded.tobytes())


@pytest.mark.parametrize('flags,payload', [(0x2000, b'bad'), (0x2040, bytes(8))])
def test_bad_camera_buffer_is_requeued_and_next_frame_survives(monkeypatch, flags, payload):
    import camera_utils.camera_server as server
    cam = V4L2MmapCamera('/synthetic', 2, 2, 30)
    cam.fd, cam.streaming = 123, True
    cam.pixel_format = _fourcc('YUYV')
    cam.buffers = [payload]
    sequence, requeued = [7], []
    monkeypatch.setattr(server.select, 'select', lambda *args: ([123], [], []))
    def ioctl(fd, request, buf):
        if request == server.VIDIOC_DQBUF:
            buf.index, buf.bytesused, buf.flags = 0, len(cam.buffers[0]), flags if sequence[0] == 7 else 0x2000
            buf.sequence = sequence[0]
            buf.timestamp.tv_sec, buf.timestamp.tv_usec = 100, sequence[0] * 1000
        elif request == server.VIDIOC_QBUF:
            requeued.append(buf.sequence)
    monkeypatch.setattr(server, '_xioctl', ioctl)
    ok, frame, _ = cam.read()
    assert not ok and frame is None and requeued == [7]
    cam.buffers[0] = bytes([100, 128, 100, 128]) * 2
    sequence[0] = 8
    ok, frame, stamp = cam.read()
    assert ok and frame.shape == (2, 2, 3) and stamp == 100.008
    assert requeued == [7, 8] and cam.last_sequence == 8


def test_d405_no_webcam_controls(tmp_path):
    cfg = resolve_camera_config('left', '/unused', params_path=str(tmp_path / 'missing.npz'),
                                width_override=640, height_override=480, fps_override=30,
                                use_default_controls=False)
    assert cfg.settings == {}
    assert (cfg.width, cfg.height, cfg.fps) == (640, 480, 30)


def test_recording_uses_exact_observation_and_30hz_deadlines():
    now = [100.]
    reads, rows = [], []
    def state():
        reads.append(now[0])
        return np.arange(14.), -np.arange(14.)
    def capture(read, names):
        return capture_observation(read, names, clock=lambda: now[0],
            snapshot_provider=lambda names, copy_images: ({}, dict(snapshot_timestamp=now[0],
                cameras={name: dict(frame_number=len(reads), capture_timestamp=now[0]-.01,
                                   consumer_received_timestamp=now[0]-.005) for name in names})))
    report = collector.record_samples(state, SimpleNamespace(append_sample=rows.append), duration=5,
        clock=lambda: now[0], sleep=lambda dt: now.__setitem__(0, now[0]+dt), capture=capture)
    assert len(reads) == len(rows) == 150
    assert report['measured_recording_hz'] == pytest.approx(30)
    for i, row in enumerate(rows):
        np.testing.assert_array_equal(row['qpos'], row['commanded_action'])
        np.testing.assert_array_equal(row['qvel'], -row['qpos'])
        assert row['camera_observation']['cameras']['video_left']['frame_number'] == i+1


def test_camera_failure_does_not_append_invalid_row():
    rows = []
    def fail(*args):
        raise RuntimeError('stale camera')
    with pytest.raises(RuntimeError, match='stale camera'):
        collector.record_samples(lambda: None, SimpleNamespace(append_sample=rows.append), duration=5,
                                 capture=fail, sleep=lambda dt: None)
    assert rows == []


@pytest.mark.parametrize('invalid', ['incomplete', 'unalignable', 'nonfinite', 'gripper'])
def test_review_rejects_invalid_take_before_motion(tmp_path, monkeypatch, invalid):
    from recording import recording_replay
    manifest = tmp_path / 'manifest.json'
    record = dict(complete=True, retained=True, alignable=True)
    if invalid == 'incomplete':
        record['complete'] = False
    if invalid == 'unalignable':
        record['alignable'] = False
    manifest.write_text(json.dumps(record))
    rows = np.zeros((2, 14))
    if invalid == 'nonfinite':
        rows[1, 2] = np.nan
    if invalid == 'gripper':
        rows[1, 13] = 1.1
    monkeypatch.setattr(collector, 'wait_key', lambda *args: 'r')
    monkeypatch.setattr(recording_replay, 'load_replay_episode',
                        lambda *args, **kw: (rows, 'video_left', np.zeros((2,2,3))))
    def forbidden(*args, **kw):
        pytest.fail('Invalid take must not reach capture or motion')
    monkeypatch.setattr(recording_replay, 'replay_take', forbidden)
    monkeypatch.setattr(collector, 'capture_observation', forbidden)
    backend = SimpleNamespace(ctx=SimpleNamespace(robot_contract=collector.ROBOT_CONTRACT, fps=30))
    with pytest.raises(ValueError):
        collector.review_recording(backend, manifest, SimpleNamespace())


def test_review_uses_current_raw_take_native_replay_and_standard_capture(tmp_path, monkeypatch):
    from recording import recording_replay
    manifest = tmp_path / 'manifest.json'
    original = json.dumps(dict(complete=True, retained=True, alignable=True))
    manifest.write_text(original)
    rows = np.zeros((3, 14))
    reference = np.zeros((2, 2, 3), dtype=np.uint8)
    live = np.ones_like(reference)
    decisions = iter(['r', '1'])
    monkeypatch.setattr(collector, 'wait_key', lambda *args: next(decisions))
    backend = SimpleNamespace(ctx=SimpleNamespace(robot_contract=collector.ROBOT_CONTRACT, fps=30),
                              read_state=lambda: None)
    def load(path, **kw):
        assert path == tmp_path
        assert kw == dict(camera_names=collector.NAMES, robot_contract=collector.ROBOT_CONTRACT, fps=30)
        return rows, 'video_left', reference
    captures = []
    def capture(read, names, *, include_images):
        assert read is backend.read_state and names == collector.NAMES and include_images
        captures.append(1)
        return SimpleNamespace(images={'video_left': live})
    def replay(robot, actions, camera, first, **kw):
        assert captures == [1]  # Live timing checked before motion.
        assert robot is backend and actions is rows and first is reference
        assert camera == 'video_left' and kw['gain_scales'] == (1., 1.)
        assert kw['live_frame'](camera) is live
        assert kw['poll_keys']() == []
        return 'completed'
    monkeypatch.setattr(recording_replay, 'load_replay_episode', load)
    monkeypatch.setattr(recording_replay, 'replay_take', replay)
    monkeypatch.setattr(collector, 'capture_observation', capture)
    assert collector.review_recording(backend, manifest, SimpleNamespace(poll=lambda: None))
    assert manifest.read_text() == original


@pytest.mark.parametrize('outcome', ['completed', 'cancelled', 'stopped', 'error'])
def test_native_replay_lifecycle_without_hardware(tmp_path, monkeypatch, outcome):
    from recording.recording_replay import replay_take
    import config.paths
    monkeypatch.setattr(config.paths, 'REPLAY_LOG_DIR', tmp_path)
    events = []
    queue = ['h'] if outcome == 'cancelled' else ['1']
    if outcome == 'stopped':
        queue += ['1', 'h']
    backend = SimpleNamespace(ctx=SimpleNamespace(fps=30),
        move_to_pose=lambda row: events.append(('move', row.copy())),
        enter_safe_stop=lambda: events.append('damping'),
        setup_for_recording=lambda: events.append('setup'),
        enter_motion_mode=lambda: events.append('hold'))
    actions = np.zeros((3, 14))
    def replay(rows, **kw):
        assert rows is actions and kw['fps'] == 30
        events.append('replay')
        if outcome == 'error':
            raise RuntimeError('simulated control failure')
        return not kw['should_stop']()
    backend.replay_rows = replay
    def run():
        return replay_take(backend, actions, None, None,
            poll_keys=lambda: [queue.pop(0)] if queue else [],
            live_frame=lambda name: None, sleep=lambda _: None, notify=lambda _: None)
    if outcome == 'error':
        with pytest.raises(RuntimeError, match='simulated control failure'):
            run()  # Must propagate to the collector's existing damping_exit.
        assert 'hold' not in events
    else:
        assert run() == outcome
        if outcome == 'cancelled':
            assert 'replay' not in events
        elif outcome == 'stopped':
            assert events[-2:] == ['damping', 'setup']
        else:
            assert events[-1] == 'hold'
    np.testing.assert_array_equal(events[0][1], actions[0])


def test_native_publication_preserves_yam_identity_and_exact_frames(tmp_path):
    import zarr
    from recording.distributed_recording import ControlWriter, publish_recording
    from data_processing.episode_data import Episode
    raw = tmp_path / 'raw'
    raw.mkdir()
    writer = ControlWriter(raw / 'control.zarr')
    for i in range(3):
        stamp = 100. + i / 30
        obs = capture_observation(lambda: (np.arange(14.) + i, np.zeros(14)), collector.NAMES,
            clock=lambda: stamp,
            snapshot_provider=lambda names, copy_images: ({}, dict(snapshot_timestamp=stamp,
                cameras={name: dict(frame_number=i, capture_timestamp=stamp-.01,
                                   consumer_received_timestamp=stamp-.005) for name in names})))
        writer.append({**obs.recording_fields(), 'commanded_action': obs.qpos.copy()})
    for name in collector.NAMES:
        directory = raw / f'{name}.zarr'
        directory.mkdir()
        video = cv2.VideoWriter(str(directory / 'main.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 30, (640, 480))
        assert video.isOpened()
        for i in range(3):
            video.write(np.full((480,640,3), 30+i*40, dtype=np.uint8))
        video.release()
        (raw / f'{name}.timing.jsonl').write_text(''.join(json.dumps(dict(
            video_frame=i,frame_number=i,capture_timestamp=100.+i/30-.01,continuity_id=0))+'\n' for i in range(3)))
    manifest = dict(complete=True,retained=True,alignable=True,sample_count=3,fps=30,
        camera_names=collector.NAMES,is_double_arm=True,rollout_dir=str(tmp_path/'published'),
        outputs={'rollout':dict(requested=True,decision='success')},
        metadata=dict(robot_type='yam',robot_model='yam_ultra_2',action_label_source='same_row_measured_qpos',
                      robot_contract=collector.ROBOT_CONTRACT))
    path = raw / 'manifest.json'
    path.write_text(json.dumps(manifest))
    (raw/'alignment_report.json').write_text(json.dumps(dict(alignable=True,errors=[],retained_robot_samples=3)))
    outputs = publish_recording(path)
    assert len(outputs)==1
    from recording.recording_replay import load_replay_episode
    actions, camera, reference = load_replay_episode(raw, camera_names=collector.NAMES,
        robot_contract=collector.ROBOT_CONTRACT, fps=30)
    assert actions.shape == (3, 14) and camera == 'video_left'
    assert reference.shape == (480, 640, 3)
    with Episode(outputs[0]) as episode:
        assert episode.attrs['robot_type']=='yam'
        assert episode.attrs['robot_model']=='yam_ultra_2'
        assert episode.qpos.shape==(3,14)
        assert episode.images['video_left'][1].shape==(480,640,3)
        np.testing.assert_array_equal(episode.actions, episode.qpos)
