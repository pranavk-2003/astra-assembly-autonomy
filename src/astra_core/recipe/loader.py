"""Recipe/correction loading: schema validation + semantic checks + typed
construction. The one place a malformed input is guaranteed to raise
RecipeValidationError rather than propagate a raw KeyError/TypeError (R1)."""
from __future__ import annotations

import json
from pathlib import Path

import jsonschema
import numpy as np

from astra_core.recipe.errors import RecipeValidationError
from astra_core.recipe.models import (
    Constraints,
    Grasp,
    Joint,
    Obstacle,
    Part,
    PerceptionCorrection,
    Recipe,
    Shape,
)
from astra_core.recipe.schema import CORRECTION_SCHEMA, RECIPE_SCHEMA
from astra_core.geometry.pose import Pose


def _read_json(source) -> dict:
    if isinstance(source, (str, Path)):
        try:
            return json.loads(Path(source).read_text())
        except json.JSONDecodeError as exc:
            raise RecipeValidationError(f"invalid JSON in {source}: {exc}") from exc
        except OSError as exc:
            raise RecipeValidationError(f"cannot read {source}: {exc}") from exc
    if isinstance(source, dict):
        return source
    raise RecipeValidationError(f"unsupported recipe source type {type(source)!r}")


def _validate(data: dict, schema: dict, kind: str) -> None:
    try:
        jsonschema.validate(instance=data, schema=schema)
    except jsonschema.ValidationError as exc:
        path = "/".join(str(p) for p in exc.path) or "<root>"
        raise RecipeValidationError(f"{kind} schema violation at {path}: {exc.message}") from exc


def _pose(d: dict) -> Pose:
    return Pose.from_xyz_rpy(d["xyz"], d["rpy"])


def _shape(d: dict) -> Shape:
    return Shape(kind=d["type"], size=np.asarray(d["size"], dtype=float))


def load_recipe(source) -> Recipe:
    data = _read_json(source)
    _validate(data, RECIPE_SCHEMA, "recipe")

    parts = tuple(
        Part(
            id=p["id"],
            shape=_shape(p["shape"]),
            source_pose=_pose(p["source_pose"]),
            assembly_pose=_pose(p["assembly_pose"]),
            grasp=Grasp(
                approach_vector=np.asarray(p["grasp"]["approach_vector"], dtype=float),
                approach_distance=float(p["grasp"]["approach_distance"]),
                grasp_offset_xyz=np.asarray(p["grasp"]["grasp_offset_xyz"], dtype=float),
            ),
        )
        for p in data["parts"]
    )

    part_ids = {p.id for p in parts}
    if len(part_ids) != len(parts):
        raise RecipeValidationError("duplicate part id in recipe")

    joints = tuple(
        Joint(
            id=j["id"],
            type=j["type"],
            members=tuple(j["members"]),
            pose=_pose(j["pose"]),
            approach_vector=np.asarray(j["approach_vector"], dtype=float),
            approach_distance=float(j["approach_distance"]),
        )
        for j in data["joints"]
    )

    for j in joints:
        unknown = [m for m in j.members if m not in part_ids]
        if unknown:
            raise RecipeValidationError(
                f"joint {j.id!r} references unknown member(s) {unknown} not present in parts"
            )

    obstacles = tuple(
        Obstacle(id=o["id"], shape=_shape(o["shape"]), pose=_pose(o["pose"]))
        for o in data["obstacles"]
    )

    constraints = Constraints(
        max_perception_translation_correction_m=float(
            data["constraints"]["max_perception_translation_correction_m"]
        ),
        max_perception_rotation_correction_deg=float(
            data["constraints"]["max_perception_rotation_correction_deg"]
        ),
    )

    return Recipe(
        job_id=data["job_id"],
        frame_id=data["frame_id"],
        units=data["units"],
        parts=parts,
        joints=joints,
        obstacles=obstacles,
        constraints=constraints,
    )


def load_correction(source) -> PerceptionCorrection:
    data = _read_json(source)
    _validate(data, CORRECTION_SCHEMA, "perception correction")
    return PerceptionCorrection(
        job_id=data["job_id"],
        part_id=data["part_id"],
        confidence=float(data["confidence"]),
        delta_translation_m=np.asarray(data["delta_translation_m"], dtype=float),
        delta_rpy_deg=np.asarray(data["delta_rpy_deg"], dtype=float),
        raw=data,
    )
