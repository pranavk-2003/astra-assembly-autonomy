"""JobContext is the orchestrator's shared blackboard: everything a step
dispatch or a future BT node would need to read/write. Kept as one explicit
object (rather than scattered locals) so a py_trees blackboard could wrap
this later without touching the skill layer (see docs/alternatives_considered.md)."""
from __future__ import annotations

from dataclasses import dataclass, field

from astra_core.orchestrator.states import JobStatus
from astra_core.recipe.models import Recipe
from astra_core.skills.base import SkillContext
from astra_core.world.world_model import WorldModel


@dataclass
class JobContext:
    recipe: Recipe
    world: WorldModel
    skill_ctx: SkillContext
    status: JobStatus | None = None
    failure_counts: dict[tuple[int, str], int] = field(default_factory=dict)

    def attempt_index(self, step_index: int, failure_class_value: str) -> int:
        return self.failure_counts.get((step_index, failure_class_value), 0)

    def record_failure(self, step_index: int, failure_class_value: str) -> int:
        key = (step_index, failure_class_value)
        self.failure_counts[key] = self.failure_counts.get(key, 0) + 1
        return self.failure_counts[key]
