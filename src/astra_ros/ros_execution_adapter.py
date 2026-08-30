"""ExecutionPort implementation: hands a planned trajectory to MoveItPy's
own executor (which talks to the real/simulated controller via ros2_control).
Gripper open/close reuses the same plan-then-execute path as the arm, against
the SRDF's own "hand" group and its "open"/"close" named states - no new
action client needed, since moveit_controller_manager already maps
panda_hand_controller as a GripperCommand action
(config/gripper_moveit_controllers.yaml, upstream). Grasp verification stays
simulated - this cell has no force/tactile sensing (R9 explicitly allows
pick-verification to be simulated) - but is now tied to the gripper-close
plan+execute actually succeeding, not to bookkeeping alone."""
from __future__ import annotations

from moveit.planning import MoveItPy

from astra_core.ports.execution_port import ExecutionPort, ExecutionResult, ExecutionStatus


class ROSExecutionAdapter(ExecutionPort):
    def __init__(
        self,
        moveit_py: MoveItPy,
        controller_names: list[str] | None = None,
        gripper_group: str = "hand",
    ) -> None:
        self._moveit_py = moveit_py
        self._controller_names = controller_names or []
        self._gripper_group = gripper_group
        self._attached_ids: set[str] = set()

    def execute(self, trajectory: object) -> ExecutionResult:
        ok = self._moveit_py.execute(trajectory, controllers=self._controller_names)
        return ExecutionResult(status=ExecutionStatus.SUCCESS if ok else ExecutionStatus.CONTROLLER_ERROR)

    def _move_gripper(self, configuration_name: str) -> bool:
        gripper = self._moveit_py.get_planning_component(self._gripper_group)
        gripper.set_start_state_to_current_state()
        gripper.set_goal_state(configuration_name=configuration_name)
        plan_result = gripper.plan()
        if not plan_result:
            return False
        return bool(self._moveit_py.execute(plan_result.trajectory, controllers=self._controller_names))

    def open_gripper(self) -> None:
        self._move_gripper("open")

    def attach(self, object_id: str) -> None:
        if self._move_gripper("close"):
            self._attached_ids.add(object_id)

    def detach(self, object_id: str) -> None:
        self._move_gripper("open")
        self._attached_ids.discard(object_id)

    def verify_grasp(self, object_id: str) -> bool:
        # No force/tactile sensing in this cell (R9 allows simulated
        # verification) - treat a successful gripper-close plan+execute as
        # verification.
        return object_id in self._attached_ids

    def stop(self) -> None:
        pass  # no protective-stop channel wired in simulation; see R12 note.
