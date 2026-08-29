"""Maps planner/execution/perception outcomes to a FailureClass. This is the
single seam where a planner- or controller-specific error gets translated
into the planner-neutral vocabulary the recovery policy understands."""
from __future__ import annotations

from astra_core.perception.gate import GateResult
from astra_core.ports.execution_port import ExecutionResult, ExecutionStatus
from astra_core.ports.planner_port import FailureReason, PlanResult
from astra_core.recovery.policy import FailureClass

_PLANNER_MAP = {
    FailureReason.IK_UNREACHABLE: FailureClass.IK_UNREACHABLE,
    FailureReason.COLLISION: FailureClass.PLANNING_COLLISION,
    FailureReason.PLANNING_TIMEOUT: FailureClass.PLANNING_COLLISION,
    FailureReason.PLANNING_FAILED: FailureClass.PLANNING_COLLISION,
}

_EXECUTION_MAP = {
    ExecutionStatus.CONTROLLER_ERROR: FailureClass.EXECUTION_ERROR,
    ExecutionStatus.PROTECTIVE_STOP: FailureClass.EXECUTION_ERROR,
    ExecutionStatus.COMM_TIMEOUT: FailureClass.COMM_TIMEOUT,
}


def classify_plan_failure(result: PlanResult) -> FailureClass:
    assert not result.success
    return _PLANNER_MAP[result.failure_reason]


def classify_execution_failure(result: ExecutionResult) -> FailureClass:
    assert not result.ok
    return _EXECUTION_MAP[result.status]


def classify_perception_rejection(gate_result: GateResult) -> FailureClass:
    assert not gate_result.accepted
    return FailureClass.PERCEPTION_REJECTED


def classify_pick_verify_failure() -> FailureClass:
    return FailureClass.PICK_VERIFY_FAILED
