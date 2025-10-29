"""
===============================================================================
Autonomous Mobile Robot Navigation Pipeline Fast test
Author: Busheng Zhang
Version: 1.0
Date: October 2025
===============================================================================
Purpose
--------
Implement Fast test on seeds on 6, 243, 463.
FULLY written by AI!
"""

import time
import math
import copy
import sys
import traceback

import matplotlib
matplotlib.use("Agg")  # disable interactive windows for automated testing

from typing import List, Tuple

import util_hainan as U
import main_hainan as M


def run_seed(seed: int, time_limit_s: float = 360.0) -> Tuple[bool, int, float]:
    """
    Run the navigation pipeline non-interactively for a given random maze seed.

    Returns (success, steps, elapsed_seconds).
    - success=True if robot reaches the goal within arrival tolerance before time limit
    - steps: number of loop iterations executed
    - elapsed_seconds: wall-clock runtime
    """

    settings = copy.deepcopy(M.DEFAULTS)
    # Force UNKNOWN navigation mode
    settings.setdefault("navigation", {})["mode"] = "UNKNOWN"
    # Fix random maze seed
    settings.setdefault("random_maze", {})["random_seed"] = seed
    # Reduce viz pause (though not rendering)
    settings.setdefault("viz", {})["pause_s"] = 0.0

    # Build app config without interactive selection
    app = settings["app"].copy()
    app["map_type"] = "RANDOM"

    # World and initial constructs
    world, entrance, goal_cell = U.build_world(settings, app)
    planner = U.create_planner(world, settings["planning"]["sample_step_m"], settings["robot"]["robot_radius_m"])
    path = U.initialise_navigation_path(planner, entrance, goal_cell, settings, "UNKNOWN")
    sensor = U.create_lidar(settings["lidar"])
    ogm = U.create_ogm(settings["ogm"], 0.0, 0.0, world["size_m"], world["size_m"])

    # State (minimal fields for non-interactive run)
    start_x, start_y = U.cell_center(entrance, world["cell_size_m"])
    start_heading = math.atan2(path[1][1] - start_y, path[1][0] - start_x) if len(path) >= 2 else 0.0
    astar_pts = planner["cspace"] if planner["cspace"] else planner["obstacles"]

    state = U.SimulationState(
        world=world,
        entrance=entrance,
        goal=U.make_goal(goal_cell),
        path=path,
        sensor=sensor,
        ogm=ogm,
        viz={"cfg": settings["viz"], "axes": {}, "size": world["size_m"], "fig": None},
        logger={"pose": None, "lidar": None, "diag": None},
        pose=U.make_pose(start_x, start_y, start_heading),
        settings=settings,
        icp_prev_pts=None,
        icp_prev_pose=None,
        step=0,
        astar_pts=astar_pts,
        ctrl=settings["setpoing_cfg"].copy(),
        planner=planner,
    )

    robot = U.load_robot_interface(state.settings)

    t0 = time.time()
    arrival_tol = settings["app"]["arrival_tolerance_m"]

    try:
        while True:
            now = time.time()
            if now - t0 >= time_limit_s:
                return False, state.step, now - t0

            # Sense
            pose = robot.get_pose(state)
            scan_data = robot.get_scan(state, pose)

            # ICP + Fusion
            curr_pts = U.icp_points(pose, scan_data, state.settings["lidar"])
            icp_pose, rmse, n_pts, tf_pts = U.icp_match_step(state.icp_prev_pts, curr_pts, state.icp_prev_pose)
            fused_pose = U.fuse_icp_pose(state.settings, pose, icp_pose, rmse, n_pts)
            if fused_pose is not None:
                pose = fused_pose
            state.icp_prev_pts, state.icp_prev_pose = curr_pts, pose
            state.pose = pose

            # Map update
            U.update_ogm(state.ogm, scan_data, pose)

            # Plan
            M.determine_navigation_path(state)
            setpoint = U.compute_setpoint(state.ctrl, state.path, pose)

            # Act
            new_pose = robot.apply_setpoint(state, pose, setpoint)
            state.pose = new_pose
            state.step += 1

            # Check goal
            goal_x, goal_y = U.cell_center(state.goal["cell"], state.world["cell_size_m"])
            if math.hypot(goal_x - new_pose["x"], goal_y - new_pose["y"]) <= arrival_tol:
                return True, state.step, time.time() - t0
    except Exception:
        traceback.print_exc()
        return False, state.step, time.time() - t0


def main():
    seeds = [6, 243, 463]
    limit_s = 150.0
    results: List[Tuple[int, bool, int, float]] = []
    for seed in seeds:
        ok, steps, elapsed = run_seed(seed, time_limit_s=limit_s)
        results.append((seed, ok, steps, elapsed))
        status = "PASS" if ok else "FAIL"
        print(f"seed={seed} | {status} | steps={steps} | time={elapsed:.1f}s")

    # Overall exit code: 0 if all pass, 1 otherwise
    for attempt_id, result in enumerate(results):
        print(f'Attempt {attempt_id}: {result}')


if __name__ == "__main__":
    main()