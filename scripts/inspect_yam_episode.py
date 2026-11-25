"""Read a saved bimanual YAM demonstration with the native reader; no hardware."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import numpy as np
from data_processing.episode_data import Episode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path)
    args = parser.parse_args()
    with Episode(args.episode) as episode:
        count = len(episode.qpos)
        if count < 2:
            raise ValueError('Expected at least two bimanual samples')
        for name in ('qpos', 'qvel', 'actions'):
            array = np.asarray(getattr(episode, name))
            if array.shape != (count, 14) or not np.isfinite(array).all():
                raise ValueError(f'{name}: expected finite ({count}, 14) array')
        if not np.array_equal(episode.qpos, episode.actions):
            raise ValueError('This hand-guided recorder requires same-row measured qpos labels')
        if set(episode.images) != {'video_left', 'video_right'}:
            raise ValueError('Expected both wrist cameras')
        indices = [0, count // 2, count - 1]
        for name, frames in episode.images.items():
            for index in indices:
                frame = frames[index]
                if frame.shape != (480, 640, 3) or frame.dtype != np.uint8:
                    raise ValueError(f'{name}[{index}]: expected uint8 480x640x3')
        timestamps = np.asarray(episode.timestamps)
        if not np.isfinite(timestamps).all() or np.any(np.diff(timestamps) <= 0):
            raise ValueError('Observation timestamps must strictly increase')
        print(json.dumps(dict(samples=count, shape=[count, 14], nominal_fps=episode.fps,
                              measured_hz=(count - 1) / float(timestamps[-1] - timestamps[0]),
                              same_row_actions=True, checked_image_indices=indices,
                              cameras=sorted(episode.images)), indent=2))
        print('PASS: native state/action and sampled image reads. Not a full-video or training acceptance test.')


if __name__ == '__main__':
    main()
