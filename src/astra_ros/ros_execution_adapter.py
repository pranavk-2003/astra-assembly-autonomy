"""ExecutionPort implementation: hands a planned trajectory to MoveItPy's
own executor (which talks to the real/simulated controller via ros2_control).
Gripper actuation and grasp verification are stubbed - this assessment's cell
has no gripper controller or force/tactile sensing configured (R9 explicitly
allows pick-verification to be simulated); see docs/industrialization.md for
the real-controller replacement."""
from __future__ import annotations

from moveit.planning import MoveItPy

from astra_core.ports.execution_port import ExecutionPort, ExecutionResult, ExecutionStatus


class ROSExecutionAdapter(ExecutionPort):
    def __init__(self, moveit_py: MoveItPy, controller_names: list[str] | None = None) -> None:
        self._moveit_py = moveit_py
        self._controller_names = controller_names or []
        self._attached_ids: set[str] = set()

    def execute(self, trajectory: object) -> ExecutionResult:
        ok = self._moveit_py.execute(trajectory, controllers=self._controller_names)
        return ExecutionResult(status=ExecutionStatus.SUCCESS if ok else ExecutionStatus.CONTROLLER_ERROR)

    def attach(self, object_id: str) -> None:
        # Real gripper close command goes here (R12: gripper action client).
        self._attached_ids.add(object_id)

    def detach(self, object_id: str) -> None:
        # Real gripper open command goes here (R12: gripper action client).
        self._attached_ids.discard(object_id)

    def verify_grasp(self, object_id: str) -> bool:
        # No force/tactile sensing in this cell (R9 allows simulated
        # verification) - treat "attach succeeded" as verification.
        return object_id in self._attached_ids

    def stop(self) -> None:
        pass  # no protective-stop channel wired in simulation; see R12 note.
