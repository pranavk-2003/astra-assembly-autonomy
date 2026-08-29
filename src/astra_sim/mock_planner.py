"""Headless PlannerPort implementation: always succeeds with a trivial
"trajectory" (the target pose itself) unless the FaultInjector has a queued
failure. Enough to drive the full orchestrator/skill/recovery path with no
ROS graph and no simulator (R11's "prefer pure-Python unit tests")."""
from __future__ import annotations

from astra_core.geometry.pose import Pose
from astra_core.ports.planner_port import PlannerPort, PlanResult
from astra_sim.fault_injector import FaultInjector

FIXED_PLAN_TIME_MS = 5.0


class MockPlanner(PlannerPort):
    def __init__(self, fault_injector: FaultInjector | None = None) -> None:
        self.fault_injector = fault_injector or FaultInjector()
        self.calls: list[Pose] = []

    def plan_to_pose(self, target: Pose, seed: int = 0) -> PlanResult:
        self.calls.append(target)
        reason = self.fault_injector.next_plan_failure()
        if reason is not None:
            return PlanResult.failed(reason, plan_time_ms=FIXED_PLAN_TIME_MS)
        return PlanResult.ok(trajectory=target, plan_time_ms=FIXED_PLAN_TIME_MS)

    def check_ik(self, target: Pose) -> bool:
        return True
