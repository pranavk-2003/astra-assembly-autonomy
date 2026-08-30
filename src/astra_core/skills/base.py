"""Skill Layer contracts. Skills are pure functions of (context, target
parameters) - the assessment spec: "Skills take a target and a context; they hold no
product geometry." No part id, joint id or coordinate is ever written inside
a skill; every skill in this package works identically for Variant A, Variant
B, or any future recipe."""
from __future__ import annotations

import time
from dataclasses import dataclass

from astra_core.geometry.pose import Pose
from astra_core.ports.execution_port import ExecutionPort, ExecutionResult
from astra_core.ports.planner_port import PlannerPort, PlanResult
from astra_core.ports.scene_port import ScenePort
from astra_core.trace.logger import TraceLogger
from astra_core.world.world_model import WorldModel


@dataclass
class SkillContext:
    planner: PlannerPort
    execution: ExecutionPort
    scene: ScenePort
    logger: TraceLogger
    job_id: str
    world: WorldModel | None = None


def obstacle_ids(ctx: "SkillContext") -> tuple[str, ...]:
    """The recipe's obstacle ids, for exempting a CARRIED part from colliding
    with world geometry the recipe deliberately puts it inside (a source pose
    within an exclusion zone, an assembly pose resting on a jig). Read from the
    world model, never written literally - a skill holds no product geometry.
    """
    if ctx.world is None:
        return ()
    return tuple(ctx.world.obstacles)


@dataclass(frozen=True)
class SkillOutcome:
    success: bool
    plan_result: PlanResult | None = None
    execution_result: ExecutionResult | None = None
    pick_verified: bool | None = None


def plan_and_execute(ctx: SkillContext, skill_name: str, target: Pose) -> SkillOutcome:
    """Shared plan->execute->log sequence every motion skill funnels through,
    so R10's trace fields are populated identically regardless of skill."""
    t0 = time.monotonic()
    plan_result = ctx.planner.plan_to_pose(target)
    plan_ms = (time.monotonic() - t0) * 1000.0

    ctx.logger.log(
        step="plan",
        skill=skill_name,
        target_xyz=target.xyz.tolist(),
        plan_success=plan_result.success,
        plan_time_ms=round(plan_ms, 3),
        failure_reason=None if plan_result.success else plan_result.failure_reason.value,
    )
    if not plan_result.success:
        return SkillOutcome(success=False, plan_result=plan_result)

    exec_result = ctx.execution.execute(plan_result.trajectory)
    ctx.logger.log(
        step="execute",
        skill=skill_name,
        target_xyz=target.xyz.tolist(),
        exec_result=exec_result.status.value,
    )
    if not exec_result.ok:
        return SkillOutcome(success=False, plan_result=plan_result, execution_result=exec_result)

    return SkillOutcome(success=True, plan_result=plan_result, execution_result=exec_result)
