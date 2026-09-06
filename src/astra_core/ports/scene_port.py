"""ScenePort: collision-world mutation contract. The MoveIt2 adapter mirrors
these calls into a PlanningScene; astra_sim's mock just records them (enough
to assert R3's attach/detach correctness in tests without a simulator)."""
from __future__ import annotations

import abc
from collections.abc import Sequence

from astra_core.geometry.pose import Pose
from astra_core.recipe.models import Shape


class ScenePort(abc.ABC):
    @abc.abstractmethod
    def add_object(self, object_id: str, shape: Shape, pose: Pose,
                   movable: bool = True) -> None:
        """Add collision geometry. `movable` distinguishes a workpiece the
        robot manipulates from fixed cell structure; a backend simulating
        physics needs it (a workpiece must be a dynamic body to be grasped,
        fixed structure must not move). Planning-only backends ignore it."""

    @abc.abstractmethod
    def update_pose(self, object_id: str, pose: Pose) -> None: ...

    @abc.abstractmethod
    def attach(self, object_id: str) -> None: ...

    @abc.abstractmethod
    def detach(self, object_id: str, pose: Pose) -> None: ...

    @abc.abstractmethod
    def allow_collision(self, object_id: str, with_ids: Sequence[str] = ()) -> None:
        """Temporarily permit the gripper to approach/overlap object_id, for
        the final grasp descent immediately before attach() (a real planner
        otherwise reports the goal pose as in collision - see
        docs/moveit2_integration_notes.md).

        `with_ids` additionally exempts object_id against those world objects.
        A carried workpiece legitimately occupies space the recipe assigns it:
        a part can rest inside an exclusion zone at its source pose and is
        assembled onto its jig at its assembly pose. Collision-checking a
        held part against those makes every plan fail from the start state
        (measured on the supplied recipes), so the recipe's own obstacle ids
        are passed here while the ARM itself keeps checking against them."""

    def observe_pose(self, object_id: str):
        """The object's measured pose, or None if this backend cannot sense.

        A backend wired to a sensor - or to a physics simulation, which is the
        same problem - answers where the object ACTUALLY is, as opposed to
        where the recipe nominally puts it. Returning None means "no
        observation available", and the caller keeps using the nominal pose.
        """
        return None

    def sync_view(self) -> None:
        """Refresh any external view of the scene after the robot has moved.

        No-op by default. A backend that mirrors the scene somewhere else (a
        simulator's own render, say) uses this to keep a carried part drawn
        where the plan actually has it, instead of frozen where it was
        grasped. Purely presentational - nothing here affects planning."""

    def allow_arm_collision(self, object_id: str) -> None:
        """Permit the WHOLE ARM, not just the gripper, to occupy object_id.

        Reserved for work-holding structure (see
        WorldModel.work_holding_obstacles): reaching a pose inside a jig puts
        the forearm through it, verified with MoveIt's collision checker
        ('panda_link5' against such an obstacle at a place pose). Kept a
        separate verb from allow_collision so the far weaker guarantee it
        gives is explicit at every call site. No-op by default."""

    def disallow_arm_collision(self, object_id: str) -> None:
        """Restore full arm collision checking against object_id."""

    @abc.abstractmethod
    def disallow_collision(self, object_id: str, with_ids: Sequence[str] = ()) -> None:
        """Re-enable collision checking of object_id against `with_ids` -
        used mid-transit as a carried part clears each obstacle it started
        inside. Never revokes the gripper exemption from allow_collision:
        once a part has been grasped, the gripper stays exempt against it for
        the rest of the job (a placed part is still in contact with the
        gripper at the retreat's start state)."""
