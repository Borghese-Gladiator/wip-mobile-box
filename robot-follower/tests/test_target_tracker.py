"""Tests for RSSI model and bearing computation (pure math, no MuJoCo)."""

import math
import pytest
from robot_follower.core.math_utils import bearing_to


class TestRSSIModel:
    """Test the RSSI distance-to-signal model logic."""

    @pytest.mark.parametrize("dist,expected_stronger", [
        (1.0, True),
        (10.0, False),
    ])
    def test_rssi_distance_relationship(self, dist, expected_stronger):
        ref_power = -40.0
        n = 2.5
        rssi = ref_power - 10.0 * n * math.log10(max(dist, 0.01))
        if expected_stronger:
            assert rssi > -60.0
        else:
            assert rssi < -60.0

    def test_rssi_monotonically_decreasing(self):
        ref_power = -40.0
        n = 2.5
        prev_rssi = float("inf")
        for dist in [0.5, 1.0, 2.0, 5.0, 10.0]:
            rssi = ref_power - 10.0 * n * math.log10(dist)
            assert rssi < prev_rssi
            prev_rssi = rssi


class TestBearingComputation:
    @pytest.mark.parametrize("robot_pos,heading,target_pos,expected_bearing", [
        ((0, 0), 0.0, (1, 0), 0.0),
        ((0, 0), 0.0, (0, 1), math.pi / 2),
        ((0, 0), math.pi / 2, (1, 0), -math.pi / 2),
    ])
    def test_bearing(self, robot_pos, heading, target_pos, expected_bearing):
        result = bearing_to(robot_pos, heading, target_pos)
        assert result == pytest.approx(expected_bearing, abs=1e-10)
