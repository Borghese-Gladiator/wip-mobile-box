"""Tests for signal filters."""

import pytest
from robot_follower.core.filters import ExponentialMovingAverage, MedianFilter, deadband, StaleTracker


class TestEMA:
    def test_first_value_is_raw(self):
        ema = ExponentialMovingAverage(alpha=0.5)
        assert ema.update(10.0) == 10.0

    def test_smooths_values(self):
        ema = ExponentialMovingAverage(alpha=0.5)
        ema.update(10.0)
        result = ema.update(20.0)
        assert result == pytest.approx(15.0)

    def test_reset(self):
        ema = ExponentialMovingAverage(alpha=0.5)
        ema.update(10.0)
        ema.reset()
        assert ema.update(5.0) == 5.0


class TestMedianFilter:
    def test_single_value(self):
        mf = MedianFilter(window=3)
        assert mf.update(5.0) == 5.0

    def test_filters_outliers(self):
        mf = MedianFilter(window=3)
        mf.update(1.0)
        mf.update(100.0)
        result = mf.update(2.0)
        assert result == 2.0


class TestDeadband:
    @pytest.mark.parametrize("val,thresh,expected", [
        (0.1, 0.5, 0.0),
        (1.0, 0.5, 1.0),
        (-0.1, 0.5, 0.0),
        (-1.0, 0.5, -1.0),
    ])
    def test_deadband(self, val, thresh, expected):
        assert deadband(val, thresh) == expected


class TestStaleTracker:
    def test_initially_stale(self):
        tracker = StaleTracker(max_age=0.5)
        assert tracker.is_stale(0.0)

    def test_fresh_after_update(self):
        tracker = StaleTracker(max_age=0.5)
        tracker.mark_updated(1.0)
        assert not tracker.is_stale(1.2)

    def test_stale_after_timeout(self):
        tracker = StaleTracker(max_age=0.5)
        tracker.mark_updated(1.0)
        assert tracker.is_stale(2.0)
