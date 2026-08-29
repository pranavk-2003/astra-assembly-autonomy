"""PlannerPort: the only contract the skill layer knows about a planner.
Per the assessment spec, the Planner Adapter is the only place cuRobo/MoveIt2 types appear.
FailureReason is a planner-neutral enum so the recovery classifier never sees
a MoveIt/cuRobo-specific error type."""
from __future__ import annotations

import abc
import enum
from dataclasses import dataclass

from astra_core.geometry.pose import Pose


class FailureReason(enum.Enum):
    NONE = "none"
    IK_UNREACHABLE = "ik_unreachable"
    COLLISION = "collision"
    PLANNING_TIMEOUT = "planning_timeout"
    PLANNING_FAILED = "planning_failed"


@dataclass(frozen=True)
class PlanResult:
    success: bool
    trajectory: object | None
    plan_time_ms: float
    failure_reason: FailureReason = FailureReason.NONE

    @staticmethod
    def ok(trajectory: object, plan_time_ms: float) -> "PlanResult":
        return PlanResult(success=True, trajectory=trajectory, plan_time_ms=plan_time_ms)

    @staticmethod
    def failed(reason: FailureReason, plan_time_ms: float = 0.0) -> "PlanResult":
        return PlanResult(success=False, trajectory=None, plan_time_ms=plan_time_ms, failure_reason=reason)


class PlannerPort(abc.ABC):
    @abc.abstractmethod
    def plan_to_pose(self, target: Pose, seed: int = 0) -> PlanResult: ...

    @abc.abstractmethod
    def check_ik(self, target: Pose) -> bool: ...
