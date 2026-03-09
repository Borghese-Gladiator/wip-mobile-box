"""Tests for state estimator."""

import pytest
from robot_follower.core.estimator import StateEstimator
from robot_follower.core.types import IMUReading


class TestStateEstimator:
    @pytest.fixture
    def estimator(self):
        return StateEstimator(wheel_radius=0.04, wheelbase=0.22)

    def test_stationary_stays_at_origin(self, estimator):
        est = estimator.update(0.0, 0.0, None, dt=0.02, timestamp=0.02)
        assert est.pose.x == pytest.approx(0.0, abs=0.001)
        assert est.pose.y == pytest.approx(0.0, abs=0.001)

    def test_forward_motion_increases_x(self, estimator):
        for i in range(100):
            est = estimator.update(5.0, 5.0, None, dt=0.02, timestamp=(i + 1) * 0.02)
        assert est.pose.x > 0.1

    def test_imu_improves_confidence(self, estimator):
        imu = IMUReading(gyro_z=0.0, timestamp=0.02)
        est = estimator.update(0.0, 0.0, imu, dt=0.02, timestamp=0.02)
        assert est.heading_confidence > 0.8

    def test_no_imu_lower_confidence(self, estimator):
        est = estimator.update(0.0, 0.0, None, dt=0.02, timestamp=0.02)
        assert est.heading_confidence < 0.7
