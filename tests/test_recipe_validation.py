"""R11 test 1: recipe validation, including malformed-input rejection (R1)."""
import pytest

from astra_core.recipe.errors import RecipeValidationError
from astra_core.recipe.loader import load_recipe

FIXTURES = "tests/fixtures"


def test_variant_a_loads():
    recipe = load_recipe("recipes/ASTRA_Pranav_Variant_A.json")
    assert recipe.job_id == "ASTRA_PRANAV_A"
    assert {p.id for p in recipe.parts} == {"member_A", "member_B"}
    assert [j.id for j in recipe.joints] == ["J1"]


def test_variant_b_loads():
    recipe = load_recipe("recipes/ASTRA_Pranav_Variant_B.json")
    assert recipe.job_id == "ASTRA_PRANAV_B"


@pytest.mark.parametrize(
    "fixture",
    [
        "missing_frame_id.json",
        "wrong_units.json",
        "unknown_joint_member.json",
        "bad_xyz_length.json",
        "not_valid_json.json",
    ],
)
def test_malformed_recipe_rejected_cleanly(fixture):
    with pytest.raises(RecipeValidationError):
        load_recipe(f"{FIXTURES}/{fixture}")


def test_duplicate_part_id_rejected():
    import copy
    import json

    data = json.load(open("recipes/ASTRA_Pranav_Variant_A.json"))
    dup = copy.deepcopy(data)
    dup["parts"][1]["id"] = dup["parts"][0]["id"]
    with pytest.raises(RecipeValidationError):
        load_recipe(dup)
