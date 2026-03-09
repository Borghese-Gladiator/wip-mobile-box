"""MuJoCo simulation runner — entry point for sim mode."""

from __future__ import annotations

import argparse
import time as _time
from pathlib import Path

import mujoco
import mujoco.viewer

from robot_follower.core.config import load_config
from robot_follower.core.types import BehaviorState
from robot_follower.core.math_utils import distance_2d
from robot_follower.core.estimator import StateEstimator
from robot_follower.core.follower_controller import FollowerController
from robot_follower.core.obstacle_avoidance import ObstacleAvoidance
from robot_follower.core.wheel_controller import WheelController
from robot_follower.core.behavior_manager import BehaviorManager
from robot_follower.core.safety_manager import SafetyManager
from .sim_robot_io import SimRobotIO
from .mujoco_target import MuJoCoTarget


def run_simulation(config: dict, gui: bool = False, verbose: bool = True) -> bool:
    """Run the person-follower simulation. Returns True if target was reached."""
    scene_path = str(Path(__file__).parent.parent.parent / "config" / "scene_planar.xml")
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # Timing constants
    sim_dt = model.opt.timestep
    control_every = 20  # ~50 Hz at 1 kHz physics
    control_dt = sim_dt * control_every

    # Build subsystems
    rc = config["robot"]
    bc = config["behavior"]
    io = SimRobotIO(model, data, config)
    target_mover = MuJoCoTarget(model, data, config)

    # State estimator — the controller's only source of self-knowledge
    estimator = StateEstimator(
        wheel_radius=rc["wheel_radius"],
        wheelbase=rc["wheelbase"],
    )
    # Seed estimator with the initial ground-truth pose (like a GPS fix at boot)
    estimator.reset(io.get_true_pose())

    follow_band = config["target"].get("follow_band", [1.0, 3.0])
    follow_distance = (follow_band[0] + follow_band[1]) / 2.0
    follower = FollowerController(
        base_speed=3.0, steering_gain=6.0, follow_distance=follow_distance,
    )
    avoidance = ObstacleAvoidance(base_speed=3.0)
    wheel_ctrl = WheelController(
        wheel_radius=rc["wheel_radius"],
        wheelbase=rc["wheelbase"],
        max_wheel_speed=rc["max_wheel_speed"],
    )
    behavior = BehaviorManager(
        follower=follower,
        avoidance=avoidance,
        wheel_ctrl=wheel_ctrl,
        avoid_weight=bc["avoid_weight"],
        escape_stall_time=bc["escape_stall_time"],
        escape_turn_speed=bc["escape_turn_speed"],
        control_dt=control_dt,
    )
    safety = SafetyManager()

    max_steps = config["simulation"]["max_steps"]
    report_every = max(1, max_steps // 20)
    success_radius = config["target"]["success_radius"]

    if verbose:
        print("Starting person-follower simulation...")
        print(f"  Physics rate: {1.0 / sim_dt:.0f} Hz")
        print(f"  Control rate: {1.0 / control_dt:.0f} Hz")
        print(f"  Max steps: {max_steps}")
        print(f"  State source: estimator (odometry + IMU)")

    viewer = None
    if gui:
        viewer = mujoco.viewer.launch_passive(model, data)

    reached = False
    try:
        for step in range(max_steps):
            sim_time = step * sim_dt
            safety.heartbeat(sim_time)

            if step % control_every == 0:
                # Move the target
                target_mover.step(control_dt)

                # --- Read raw sensors ---
                wheels = io.read_wheels()
                imu = io.read_imu()

                # --- State estimation (replaces ground-truth pose) ---
                estimate = estimator.update(
                    left_speed=wheels.left_speed,
                    right_speed=wheels.right_speed,
                    imu=imu,
                    dt=control_dt,
                    timestamp=sim_time,
                )
                est_pose = estimate.pose

                # --- Pose-dependent sensors use estimated pose ---
                target_meas = io.read_target(est_pose)
                obs_meas = io.read_obstacles(est_pose)

                # --- Goal check uses ground truth (sim-level, not controller) ---
                true_pose = io.get_true_pose()
                target_pos = target_mover.get_position()
                true_dist = distance_2d((true_pose.x, true_pose.y), target_pos)
                at_goal = true_dist < success_radius

                # --- Behavior arbitration ---
                cmd = behavior.update(
                    target=target_meas,
                    obstacles=obs_meas,
                    robot_speed=estimate.twist.linear,
                    target_reached=at_goal,
                    sim_time=sim_time,
                )

                status = safety.check(estimate.twist.linear, sim_time)
                cmd = safety.gate_command(cmd, status)
                io.send_wheel_command(cmd)

            mujoco.mj_step(model, data)

            if viewer is not None:
                viewer.sync()
                _time.sleep(1 / 240)

            if verbose and step % report_every == 0:
                true_p = io.get_true_pose()
                est_p = estimator._odom.pose  # for debug comparison
                tp = target_mover.get_position()
                d = distance_2d((true_p.x, true_p.y), tp)
                drift = distance_2d((true_p.x, true_p.y), (est_p.x, est_p.y))
                print(
                    f"step {step:6d} | "
                    f"true=({true_p.x:5.2f}, {true_p.y:5.2f}) | "
                    f"est=({est_p.x:5.2f}, {est_p.y:5.2f}) | "
                    f"drift={drift:.3f}m | "
                    f"target=({tp[0]:5.2f}, {tp[1]:5.2f}) | "
                    f"dist={d:.2f}m | "
                    f"state={behavior.state.name}"
                )

            if behavior.state == BehaviorState.TARGET_REACHED:
                reached = True
                break
    finally:
        if viewer is not None:
            viewer.close()

    if verbose:
        t = step * sim_dt
        true_p = io.get_true_pose()
        est_p = estimator._odom.pose
        drift = distance_2d((true_p.x, true_p.y), (est_p.x, est_p.y))
        print(f"{'Target reached' if reached else 'Simulation ended'} at step {step} (t={t:.1f}s)")
        print(f"  Final odometry drift: {drift:.3f}m")

    return reached


def main() -> None:
    parser = argparse.ArgumentParser(description="Robot follower — MuJoCo simulation")
    parser.add_argument("--config", type=str, default=None, help="Path to config YAML")
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config)
    if args.max_steps is not None:
        config["simulation"]["max_steps"] = args.max_steps

    gui = args.gui and not args.headless
    run_simulation(config, gui=gui, verbose=True)


if __name__ == "__main__":
    main()
