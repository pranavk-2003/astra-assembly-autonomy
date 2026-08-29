"""Expands a Recipe into the ordered step list: pick+place per part (in
recipe order), then approach for every joint. The sequence is DERIVED from
recipe contents (how many parts, how many joints) - it is never hand-written
per job (the assessment spec's central constraint)."""
from __future__ import annotations

import enum
from dataclasses import dataclass

from astra_core.recipe.models import Recipe


class StepKind(enum.Enum):
    PICK_APPROACH = "pick_approach"
    PICK = "pick"
    PICK_RETREAT = "pick_retreat"
    PLACE_APPROACH = "place_approach"
    PLACE = "place"
    PLACE_RETREAT = "place_retreat"
    JOINT_APPROACH = "joint_approach"


@dataclass(frozen=True)
class Step:
    kind: StepKind
    part_id: str | None = None
    joint_id: str | None = None

    @property
    def label(self) -> str:
        ref = self.part_id or self.joint_id or "?"
        return f"{ref}:{self.kind.value}"


def build_steps(recipe: Recipe) -> list[Step]:
    steps: list[Step] = []
    for part in recipe.parts:
        steps.extend(
            [
                Step(StepKind.PICK_APPROACH, part_id=part.id),
                Step(StepKind.PICK, part_id=part.id),
                Step(StepKind.PICK_RETREAT, part_id=part.id),
                Step(StepKind.PLACE_APPROACH, part_id=part.id),
                Step(StepKind.PLACE, part_id=part.id),
                Step(StepKind.PLACE_RETREAT, part_id=part.id),
            ]
        )
    for joint in recipe.joints:
        steps.append(Step(StepKind.JOINT_APPROACH, joint_id=joint.id))
    return steps
