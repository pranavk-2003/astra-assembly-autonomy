"""Headless demo entrypoint: runs one recipe (optionally with a perception
correction) through the full orchestrator using the astra_sim mocks - no ROS,
no simulator. Proves R7 (same code, data-only change) and gives a quick way
to inspect the R10 trace log."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from astra_core.orchestrator.job_runner import run_job
from astra_core.recipe.loader import load_correction, load_recipe
from astra_core.skills.base import SkillContext
from astra_core.trace.logger import TraceLogger
from astra_sim.mock_executor import MockExecutor
from astra_sim.mock_planner import MockPlanner
from astra_sim.mock_scene import MockScene


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ASTRA headless job runner")
    parser.add_argument("--recipe", required=True, type=Path)
    parser.add_argument("--correction", type=Path, default=None)
    parser.add_argument("--planner", choices=["mock"], default="mock")
    parser.add_argument(
        "--confidence-threshold", type=float, default=0.90,
        help="perception gate confidence threshold (default 0.90)",
    )
    args = parser.parse_args(argv)

    recipe = load_recipe(args.recipe)
    correction = load_correction(args.correction) if args.correction else None

    logger = TraceLogger(job_id=recipe.job_id, out_path=Path("logs") / f"{recipe.job_id}.jsonl")
    ctx = SkillContext(
        planner=MockPlanner(),
        execution=MockExecutor(),
        scene=MockScene(),
        logger=logger,
        job_id=recipe.job_id,
    )

    result = run_job(recipe, ctx, correction=correction, confidence_threshold=args.confidence_threshold)
    print(f"job {recipe.job_id}: {result.status.value} "
          f"({result.steps_completed}/{result.steps_total} steps)", file=sys.stderr)
    return 0 if result.status.value == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
