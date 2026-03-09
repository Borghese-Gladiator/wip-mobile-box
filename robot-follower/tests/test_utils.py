"""Tests for core math utilities."""

import math
import pytest
from robot_follower.core.math_utils import normalize_angle, clamp, bearing_to, distance_2d


class TestNormalizeAngle:
    @pytest.mark.parametrize("input_val,expected", [
        (0.0, 0.0),
        (math.pi, math.pi),
        (-math.pi, -math.pi),
        (2 * math.pi, 0.0),
        (-2 * math.pi, 0.0),
        (3 * math.pi, math.pi),
        (-3 * math.pi, -math.pi),
        (math.pi + 0.1, -math.pi + 0.1),
    ])
    def test_normalize(self, input_val, expected):
        assert normalize_angle(input_val) == pytest.approx(expected, abs=1e-10)


class TestClamp:
    @pytest.mark.parametrize("val,lo,hi,expected", [
        (5.0, 0.0, 10.0, 5.0),
        (-1.0, 0.0, 10.0, 0.0),
        (15.0, 0.0, 10.0, 10.0),
        (0.0, 0.0, 0.0, 0.0),
    ])
    def test_clamp(self, val, lo, hi, expected):
        assert clamp(val, lo, hi) == expected


class TestBearingTo:
    def test_target_directly_ahead(self):
        assert bearing_to((0, 0), 0.0, (1, 0)) == pytest.approx(0.0, abs=1e-10)

    def test_target_to_left(self):
        assert bearing_to((0, 0), 0.0, (0, 1)) == pytest.approx(math.pi / 2, abs=1e-10)

    def test_target_behind(self):
        assert abs(bearing_to((0, 0), 0.0, (-1, 0))) == pytest.approx(math.pi, abs=1e-10)


class TestDistance2D:
    def test_same_point(self):
        assert distance_2d((0.0, 0.0), (0.0, 0.0)) == 0.0

    def test_known_distance(self):
        assert distance_2d((0.0, 0.0), (3.0, 4.0)) == pytest.approx(5.0)
