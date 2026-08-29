"""Deterministic fault injection for the mock planner/executor: pop a queued
failure on each call until exhausted, then behave normally. No randomness -
tests assert an exact recovery sequence (per the assessment spec, "the choice must be
deterministic, not incidental")."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from astra_core.ports.execution_port import ExecutionStatus
from astra_core.ports.planner_port import FailureReason


@dataclass
class FaultInjector:
    plan_failures: deque = field(default_factory=deque)
    exec_failures: deque = field(default_factory=deque)
    pick_verify_failures: int = 0

    @staticmethod
    def with_plan_failures(*reasons: FailureReason) -> "FaultInjector":
        return FaultInjector(plan_failures=deque(reasons))

    @staticmethod
    def with_exec_failures(*statuses: ExecutionStatus) -> "FaultInjector":
        return FaultInjector(exec_failures=deque(statuses))

    def next_plan_failure(self) -> FailureReason | None:
        return self.plan_failures.popleft() if self.plan_failures else None

    def next_exec_failure(self) -> ExecutionStatus | None:
        return self.exec_failures.popleft() if self.exec_failures else None

    def next_pick_verify_result(self) -> bool:
        if self.pick_verify_failures > 0:
            self.pick_verify_failures -= 1
            return False
        return True
