"""Offline install/model check: no robot constructor, CAN socket or camera opened."""
import hashlib
from importlib.metadata import version
import inspect
import json
from pathlib import Path
import subprocess
import sys

import numpy as np


def main():
    root = Path(__file__).resolve().parents[1]
    if sys.version_info[:3] != (3, 11, 15):
        raise SystemExit('Expected Python 3.11.15')
    for line in (root / 'scripts/requirements-yam-lock.txt').read_text().splitlines():
        if line and not line.startswith('#'):
            name, expected = line.split('==')
            actual = version(name)
            if actual != expected:
                raise SystemExit(f'{name}: expected {expected}, found {actual}')
    import i2rt
    from i2rt.robots.get_robot import get_yam_robot, get_yam_robots  # noqa: F401
    from i2rt.robots.kinematics import Kinematics
    from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml
    from recording.distributed_recording import DistributedRecorder  # noqa: F401
    from data_processing.episode_data import Episode  # noqa: F401
    from robologger.loggers.video_logger import VideoLogger

    sdk = Path(i2rt.__file__).resolve().parent.parent
    manifest = json.loads((root / 'scripts/patches/i2rt-source.json').read_text())
    for name, hashes in manifest['files'].items():
        if hashlib.sha256((sdk / name).read_bytes()).hexdigest() != hashes['patched_sha256']:
            raise SystemExit(f'SDK patch differs from validated source: {name}')
    source = inspect.getsource(VideoLogger)
    if not all(key in source for key in ('encoder_preset', 'encoder_threads', 'filter_threads', 'encoder_tune')):
        raise SystemExit('Apply scripts/apply_yam_robologger_patch.py first')
    encoders = subprocess.check_output(['ffmpeg', '-hide_banner', '-encoders'], text=True, stderr=subprocess.STDOUT)
    if 'libx264' not in encoders:
        raise SystemExit('FFmpeg must include libx264')
    model = combine_arm_and_gripper_xml(ArmType.YAM_ULTRA_2, GripperType.LINEAR_4310)
    pose = Kinematics(model, 'grasp_site').fk(np.zeros(8))
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise SystemExit('YAM Ultra 2 + linear_4310 model failed FK')
    print(f'PASS: pinned dependencies, SDK patches, FFmpeg and model FK. SDK={sdk}')
    print('No camera/CAN/controller was opened. Hardware acceptance is operator-run.')


if __name__ == '__main__':
    main()
