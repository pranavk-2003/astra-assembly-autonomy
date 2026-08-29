"""Headless ExecutionPort implementation. Tracks attach/detach and grasp-
verify state so tests can assert R3 correctness without a real controller."""
from __future__ import annotations

from astra_core.ports.execution_port import ExecutionPort, ExecutionResult, ExecutionStatus
from astra_sim.fault_injector import FaultInjector


class MockExecutor(ExecutionPort):
    def __init__(self, fault_injector: FaultInjector | None = None) -> None:
        self.fault_injector = fault_injector or FaultInjector()
        self.attached_ids: set[str] = set()
        self.executed: list[object] = []

    def execute(self, trajectory: object) -> ExecutionResult:
        self.executed.append(trajectory)
        status = self.fault_injector.next_exec_failure()
        return ExecutionResult(status=status or ExecutionStatus.SUCCESS)

    def attach(self, object_id: str) -> None:
        self.attached_ids.add(object_id)

    def detach(self, object_id: str) -> None:
        self.attached_ids.discard(object_id)

    def verify_grasp(self, object_id: str) -> bool:
        return self.fault_injector.next_pick_verify_result()

    def stop(self) -> None:
        pass
