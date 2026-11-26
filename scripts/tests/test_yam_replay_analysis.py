"""Offline statistics: units, signed error and incomplete/invalid segments."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('analyze_yam_replay',
    Path(__file__).parents[1] / 'analyze_yam_replay.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_angles_and_grippers_have_distinct_units():
    target = np.zeros((3, 14))
    actual = target.copy()
    actual[:, 1] = np.deg2rad([-1., 0., 2.])
    actual[:, 6] = .02
    report = module.summarize(target, actual, [10., 10.5, 11.], [4, 5, 6])
    joint, grip = report['metrics'][1], report['metrics'][6]
    assert joint['mae'] == pytest.approx(1.)
    assert joint['rmse'] == pytest.approx(np.sqrt(5/3))
    assert joint['bias'] == pytest.approx(1/3)
    assert joint['peak_row'] == 6
    assert grip['mae'] == pytest.approx(2.)
    assert grip['unit'] == 'percentage_points'
    assert report['measured_log_hz'] == 2.


@pytest.mark.parametrize('times,rows', [([1., 1., 2.], [0, 1, 2]),
    ([1., 2., 3.], [0, 2, 3]), ([1., np.nan, 3.], [0, 1, 2])])
def test_invalid_segment_not_silently_resampled(times, rows):
    with pytest.raises(ValueError):
        module.summarize(np.zeros((3, 14)), np.zeros((3, 14)), times, rows)
