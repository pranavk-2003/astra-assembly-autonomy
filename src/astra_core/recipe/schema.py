"""JSON Schema for the recipe format, as supplied by the assessment spec."""

_POSE = {
    "type": "object",
    "required": ["xyz", "rpy"],
    "properties": {
        "xyz": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
        "rpy": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
    },
    "additionalProperties": False,
}

_SHAPE = {
    "type": "object",
    "required": ["type", "size"],
    "properties": {
        "type": {"const": "box"},
        "size": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
    },
    "additionalProperties": False,
}

_GRASP = {
    "type": "object",
    "required": ["approach_vector", "approach_distance", "grasp_offset_xyz"],
    "properties": {
        "approach_vector": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
        "approach_distance": {"type": "number", "exclusiveMinimum": 0},
        "grasp_offset_xyz": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
    },
    "additionalProperties": False,
}

_PART = {
    "type": "object",
    "required": ["id", "shape", "source_pose", "assembly_pose", "grasp"],
    "properties": {
        "id": {"type": "string", "minLength": 1},
        "shape": _SHAPE,
        "source_pose": _POSE,
        "assembly_pose": _POSE,
        "grasp": _GRASP,
    },
    "additionalProperties": False,
}

_JOINT = {
    "type": "object",
    "required": ["id", "type", "members", "pose", "approach_vector", "approach_distance"],
    "properties": {
        "id": {"type": "string", "minLength": 1},
        "type": {"type": "string", "minLength": 1},
        "members": {"type": "array", "items": {"type": "string"}, "minItems": 1},
        "pose": _POSE,
        "approach_vector": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
        "approach_distance": {"type": "number", "exclusiveMinimum": 0},
    },
    "additionalProperties": False,
}

_OBSTACLE = {
    "type": "object",
    "required": ["id", "shape", "pose"],
    "properties": {
        "id": {"type": "string", "minLength": 1},
        "shape": _SHAPE,
        "pose": _POSE,
    },
    "additionalProperties": False,
}

_CONSTRAINTS = {
    "type": "object",
    "required": [
        "max_perception_translation_correction_m",
        "max_perception_rotation_correction_deg",
    ],
    "properties": {
        "max_perception_translation_correction_m": {"type": "number", "exclusiveMinimum": 0},
        "max_perception_rotation_correction_deg": {"type": "number", "exclusiveMinimum": 0},
    },
    "additionalProperties": False,
}

RECIPE_SCHEMA = {
    "type": "object",
    "required": ["job_id", "frame_id", "units", "parts", "joints", "obstacles", "constraints"],
    "properties": {
        "job_id": {"type": "string", "minLength": 1},
        "frame_id": {"const": "world"},
        "units": {"const": "m"},
        "parts": {"type": "array", "items": _PART, "minItems": 1},
        "joints": {"type": "array", "items": _JOINT},
        "obstacles": {"type": "array", "items": _OBSTACLE},
        "constraints": _CONSTRAINTS,
    },
    "additionalProperties": False,
}

CORRECTION_SCHEMA = {
    "type": "object",
    "required": ["job_id", "part_id", "confidence", "delta_translation_m", "delta_rpy_deg"],
    "properties": {
        "job_id": {"type": "string", "minLength": 1},
        "part_id": {"type": "string", "minLength": 1},
        "confidence": {"type": "number", "minimum": 0, "maximum": 1},
        "delta_translation_m": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
        "delta_rpy_deg": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
    },
    "additionalProperties": True,
}
