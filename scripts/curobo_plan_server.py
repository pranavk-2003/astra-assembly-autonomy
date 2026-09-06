"""cuRobo planning service.

Runs in cuRobo's own virtualenv and answers plan requests over stdin/stdout as
one JSON object per line. It exists as a separate process for a hard reason:
cuRobo's environment is Python 3.11 while ROS 2 Jazzy is 3.12, so rclpy's C
extension cannot be imported alongside cuRobo - the two genuinely cannot share
an interpreter. Keeping the GPU planner behind a process boundary is also how
one is usually deployed, so this is not merely a workaround.

Protocol - one JSON object per line, in and out.

  request  {"cmd": "plan", "goal": {"xyz": [..], "quat_wxyz": [..]},
            "start": [q0..qn],
            "obstacles": [{"name": .., "dims": [..], "pose": [x,y,z,qw,qx,qy,qz]}]}
  response {"ok": true, "trajectory": [[q..], ..], "dt": 0.025, "plan_ms": 12.3}
           {"ok": false, "reason": "..."}

  request  {"cmd": "ping"}      -> {"ok": true, "robot": "...", "joints": [...]}
  request  {"cmd": "shutdown"}  -> exits
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import traceback


def _emit(payload: dict) -> None:
    sys.stdout.write(json.dumps(payload) + "\n")
    sys.stdout.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description="cuRobo planning service")
    parser.add_argument("--robot", default="ur10e.yml",
                        help="cuRobo robot config (shipped name or path to a yml)")
    parser.add_argument("--ik-seeds", type=int, default=64,
                        help="IK seeds per solve; more trades plan time for a "
                             "far lower chance of a spurious 'no solution'")
    parser.add_argument("--trajopt-seeds", type=int, default=8,
                        help="trajectory-optimiser seeds per solve")
    parser.add_argument("--max-attempts", type=int, default=10,
                        help="solver retries within one plan request")
    parser.add_argument("--graph-attempts", type=int, default=2,
                        help="attempts after which the graph planner is used "
                             "to escape a local minimum")
    args = parser.parse_args()

    # Imported here so an import failure is reported over the protocol rather
    # than killing the process before the caller can see why.
    try:
        import torch
        from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
        from curobo.scene import Cuboid, Scene
        from curobo.types import GoalToolPose, JointState
    except Exception as exc:  # pragma: no cover - environment problem
        _emit({"ok": False, "reason": f"import failed: {exc}"})
        return 1

    try:
        # A collision world must be allocated up front: without scene_model the
        # planner has no collision model at all and update_world() fails with
        # "'NoneType' object has no attribute 'load_collision_model'".
        # collision_cache pre-allocates slots for the obstacles the recipe will
        # push in later - the count is a ceiling, not a fixed set.
        # Seed counts are the cuRobo equivalent of MoveIt's planning_attempts /
        # planning_time budget (config/moveit_cpp.yaml). Both solvers here are
        # randomised: cuRobo draws IK seeds and trajectory-optimiser seeds, and
        # a single draw that lands in a local minimum reports "no solution" for
        # a goal that is perfectly reachable. Measured on the Variant B place
        # and retreat goals, the shipped defaults solved as little as 1 attempt
        # in 8 for a pose the arm can plainly reach, which the recovery policy
        # then saw as ik_unreachable three times over and aborted the job.
        cfg = MotionPlannerCfg.create(
            robot=args.robot,
            scene_model="collision_table.yml",
            collision_cache={"obb": 32, "mesh": 8},
            num_ik_seeds=args.ik_seeds,
            num_trajopt_seeds=args.trajopt_seeds,
        )
        planner = MotionPlanner(cfg)
    except Exception as exc:
        _emit({"ok": False, "reason": f"planner init failed: {exc}"})
        return 1

    joint_names = list(planner.joint_names)
    _emit({"ok": True, "event": "ready", "robot": args.robot, "joints": joint_names,
           "tool_frames": list(planner.tool_frames)})

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            request = json.loads(line)
        except json.JSONDecodeError as exc:
            _emit({"ok": False, "reason": f"bad json: {exc}"})
            continue

        cmd = request.get("cmd")
        if cmd == "shutdown":
            return 0
        if cmd == "ping":
            _emit({"ok": True, "robot": args.robot, "joints": joint_names})
            continue
        if cmd != "plan":
            _emit({"ok": False, "reason": f"unknown cmd {cmd!r}"})
            continue

        try:
            obstacles = request.get("obstacles") or []
            if obstacles:
                planner.update_world(
                    Scene(cuboid=[
                        Cuboid(name=o["name"], dims=list(o["dims"]), pose=list(o["pose"]))
                        for o in obstacles
                    ])
                )

            start = JointState.from_position(
                torch.tensor([request["start"]], dtype=torch.float32, device="cuda"),
                joint_names=joint_names,
            )
            goal = request["goal"]
            goal_pose = GoalToolPose(
                tool_frames=list(planner.tool_frames),
                # Shape is [batch, horizon, links, goalset, 3|4] - a single
                # goal for a single tool frame is all leading dims = 1.
                position=torch.tensor(
                    goal["xyz"], dtype=torch.float32, device="cuda"
                ).view(1, 1, 1, 1, 3),
                quaternion=torch.tensor(
                    goal["quat_wxyz"], dtype=torch.float32, device="cuda"
                ).view(1, 1, 1, 1, 4),
            )

            t0 = time.time()
            result = planner.plan_pose(
                goal_pose, start,
                max_attempts=args.max_attempts,
                enable_graph_attempt=args.graph_attempts,
            )
            plan_ms = (time.time() - t0) * 1000.0

            if result is None or not bool(getattr(result, "success", False)):
                _emit({"ok": False, "reason": "no solution", "plan_ms": plan_ms})
                continue

            # The solution is exposed either as a JointState or as a raw
            # tensor depending on which field is present; accept both.
            traj = getattr(result, "interpolated_solution", None)
            if traj is None:
                traj = result.solution
            tensor = getattr(traj, "position", traj)
            positions = tensor.squeeze().detach().cpu().numpy().tolist()
            if positions and not isinstance(positions[0], list):
                positions = [positions]
            _emit({
                "ok": True,
                "trajectory": positions,
                "joints": joint_names,
                "dt": float(getattr(result, "interpolation_dt", 0.025)),
                "plan_ms": plan_ms,
            })
        except Exception as exc:
            _emit({"ok": False, "reason": f"{type(exc).__name__}: {exc}",
                   "trace": traceback.format_exc(limit=3)})

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
