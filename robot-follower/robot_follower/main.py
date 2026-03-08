"""Simulation entry point."""

from __future__ import annotations

import argparse
import time as _time
from pathlib import Path

import mujoco
import mujoco.viewer

from .utils import load_config
from .robot_interface import RobotInterface
from .target_tracker import TargetTracker
from .obstacle_sensors import ObstacleSensors
from .controller import Controller
from .behavior_manager import BehaviorManager
from .sim_env import SimEnvironment


def run_simulation(config: dict, gui: bool = False, verbose: bool = True) -> bool:
    """Run the person-follower simulation. Returns True if target was reached."""
    scene_path = str(Path(__file__).parent.parent / "config" / "scene.xml")

    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)

    # Forward to settle initial state
    mujoco.mj_forward(model, data)

    # Initialize subsystems
    robot = RobotInterface(model, data, config)
    tracker = TargetTracker(model, data, config)
    obstacles = ObstacleSensors(model, data, config)
    controller = Controller(config)
    behavior = BehaviorManager(config, controller)
    env = SimEnvironment(model, data, config)

    max_steps = config["simulation"]["max_steps"]
    sim_dt = model.opt.timestep
    # Control at ~50 Hz (every 20 physics steps at 1 kHz)
    control_every = 20
    report_every = max(1, max_steps // 20)

    if verbose:
        print("Starting person-follower simulation...")
        print(f"  Physics rate: {1.0/sim_dt:.0f} Hz")
        print(f"  Control rate: {1.0/(sim_dt * control_every):.0f} Hz")
        print(f"  Max steps: {max_steps}")

    viewer = None
    if gui:
        viewer = mujoco.viewer.launch_passive(model, data)

    reached = False
    try:
        for step in range(max_steps):
            sim_time = step * sim_dt

            # --- Control update ---
            if step % control_every == 0:
                # Update target motion
                env.step_target_motion(sim_dt * control_every)

                # Read sensors
                pos = robot.get_position()
                heading = robot.get_heading()
                bearing = tracker.get_bearing(heading)
                strength = tracker.get_signal_strength()
                obs_reading = obstacles.read(pos, heading)
                lin_vel, _ = robot.get_velocity()

                # Behavior arbitration → wheel velocities
                left_vel, right_vel = behavior.update(
                    bearing=bearing,
                    signal_strength=strength,
                    obstacles=obs_reading,
                    robot_speed=lin_vel,
                    target_reached=env.is_target_reached(),
                    sim_time=sim_time,
                )
                robot.set_wheel_velocities(left_vel, right_vel)

            # Step physics
            mujoco.mj_step(model, data)

            # GUI sync
            if viewer is not None:
                viewer.sync()
                _time.sleep(1 / 240)

            # Logging
            if verbose and step % report_every == 0:
                rx, ry, yaw = robot.get_pose()
                tx, ty = env.get_target_pos()
                dist = tracker.get_distance()
                print(
                    f"step {step:6d} | robot=({rx:5.2f}, {ry:5.2f}) | "
                    f"target=({tx:5.2f}, {ty:5.2f}) | dist={dist:.2f}m | "
                    f"state={behavior.state.name}"
                )

            # Check goal
            if env.is_target_reached():
                reached = True
                break
    finally:
        if viewer is not None:
            viewer.close()

    if verbose:
        if reached:
            print(f"Target reached at step {step} (t={step * sim_dt:.1f}s)!")
        else:
            print(f"Simulation ended at step {step} (t={step * sim_dt:.1f}s)")

    return reached


def main() -> None:
    parser = argparse.ArgumentParser(description="Differential-drive robot person follower")
    parser.add_argument("--config", type=str, default=None, help="Path to config YAML")
    parser.add_argument("--max-steps", type=int, default=None, help="Override max simulation steps")
    parser.add_argument("--headless", action="store_true", help="Run without GUI (default)")
    parser.add_argument("--gui", action="store_true", help="Run with MuJoCo viewer")
    args = parser.parse_args()

    config = load_config(args.config)

    if args.max_steps is not None:
        config["simulation"]["max_steps"] = args.max_steps

    gui = args.gui and not args.headless
    run_simulation(config, gui=gui, verbose=True)


if __name__ == "__main__":
    main()
