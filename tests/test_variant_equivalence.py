"""R11 test 5 (part B): Variant A and Variant B run through the identical
orchestrator/skill/planner code path and produce different, recipe-derived
target sets - the assessment's primary acceptance criterion (R7)."""
import numpy as np

from astra_core.orchestrator.job_runner import run_job
from astra_core.recipe.loader import load_recipe
from astra_core.skills.base import SkillContext
from astra_core.trace.logger import TraceLogger
from astra_sim.mock_executor import MockExecutor
from astra_sim.mock_planner import MockPlanner
from astra_sim.mock_scene import MockScene


def _run(variant: str):
    recipe = load_recipe(f"recipes/ASTRA_Pranav_Variant_{variant}.json")
    planner = MockPlanner()
    ctx = SkillContext(
        planner=planner,
        execution=MockExecutor(),
        scene=MockScene(),
        logger=TraceLogger(job_id=recipe.job_id),
        job_id=recipe.job_id,
    )
    result = run_job(recipe, ctx)
    return recipe, planner, result


def test_both_variants_complete_via_identical_code_path():
    recipe_a, planner_a, result_a = _run("A")
    recipe_b, planner_b, result_b = _run("B")

    assert result_a.status.value == "complete"
    assert result_b.status.value == "complete"
    # same step count formula: 6 per part + 1 per joint, for identical code
    assert result_a.steps_total == result_b.steps_total == 6 * 2 + 1

    targets_a = np.array([p.xyz for p in planner_a.calls])
    targets_b = np.array([p.xyz for p in planner_b.calls])
    assert targets_a.shape == targets_b.shape
    assert not np.allclose(targets_a, targets_b), "variants must plan to different, recipe-derived targets"


def test_variant_b_targets_are_derived_from_its_own_recipe_not_hardcoded():
    recipe_b, planner_b, _ = _run("B")
    member_a_assembly_xyz = recipe_b.part("member_A").assembly_pose.xyz
    # at least one planned target must land at (or standoff from) the
    # recipe's own assembly pose - proving targets trace back to THIS input
    hits = [
        p.xyz for p in planner_b.calls
        if np.allclose(p.xyz[:2], member_a_assembly_xyz[:2], atol=1e-6)
    ]
    assert hits, "no planned target traced back to variant B's own recipe geometry"
