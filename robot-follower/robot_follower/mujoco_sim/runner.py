"""MuJoCo simulation runner — entry point for sim mode."""

from __future__ import annotations

import argparse
import time as _time
from pathlib import Path

import mujoco
import mujoco.viewer

from robot_follower.core.config import load_config, merge_configs
from robot_follower.core.types import BehaviorState
from robot_follower.core.math_utils import distance_2d
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

    # Build subsystems
    rc = config["robot"]
    bc = config["behavior"]
    io = SimRobotIO(model, data, config)
    target = MuJoCoTarget(model, data, config)

    follower = FollowerController(base_speed=3.0, steering_gain=6.0)
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
    )
    safety = SafetyManager()

    max_steps = config["simulation"]["max_steps"]
    sim_dt = model.opt.timestep
    control_every = 20  # ~50 Hz at 1 kHz physics
    report_every = max(1, max_steps // 20)
    success_radius = config["target"]["success_radius"]

    if verbose:
        print("Starting person-follower simulation...")
        print(f"  Physics rate: {1.0 / sim_dt:.0f} Hz")
        print(f"  Control rate: {1.0 / (sim_dt * control_every):.0f} Hz")
        print(f"  Max steps: {max_steps}")

    viewer = None
    if gui:
        viewer = mujoco.viewer.launch_passive(model, data)

    reached = False
    try:
        for step in range(max_steps):
            sim_time = step * sim_dt
            safety.heartbeat(sim_time)

            if step % control_every == 0:
                target.step(sim_dt * control_every)

                target_meas = io.read_target()
                obs_meas = io.read_obstacles()
                vel = io.read_velocity()

                robot_pos = io.read_pose()
                target_pos = target.get_position()
                dist = distance_2d((robot_pos.x, robot_pos.y), target_pos)
                at_goal = dist < success_radius

                cmd = behavior.update(
                    target=target_meas,
                    obstacles=obs_meas,
                    robot_speed=vel.linear,
                    target_reached=at_goal,
                    sim_time=sim_time,
                )

                status = safety.check(vel.linear, sim_time)
                cmd = safety.gate_command(cmd, status)
                io.send_wheel_command(cmd)

            mujoco.mj_step(model, data)

            if viewer is not None:
                viewer.sync()
                _time.sleep(1 / 240)

            if verbose and step % report_every == 0:
                p = io.read_pose()
                tp = target.get_position()
                d = distance_2d((p.x, p.y), tp)
                print(
                    f"step {step:6d} | robot=({p.x:5.2f}, {p.y:5.2f}) | "
                    f"target=({tp[0]:5.2f}, {tp[1]:5.2f}) | dist={d:.2f}m | "
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
        print(f"{'Target reached' if reached else 'Simulation ended'} at step {step} (t={t:.1f}s)")

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
