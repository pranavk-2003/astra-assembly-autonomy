"""R11 test 4: perception gate - the supplied correction accepts, the supplied
failure injection rejects, on the exact confidence/translation/rotation
bounds from the recipe's constraints block."""
from astra_core.perception.gate import evaluate
from astra_core.recipe.loader import load_correction, load_recipe


def test_supplied_perception_correction_is_accepted():
    recipe = load_recipe("recipes/ASTRA_Pranav_Variant_B.json")
    correction = load_correction("recipes/ASTRA_Pranav_Perception_Correction.json")
    result = evaluate(correction, recipe.constraints)
    assert result.accepted
    assert result.reasons == ()


def test_supplied_failure_injection_is_rejected_on_all_three_bounds():
    recipe = load_recipe("recipes/ASTRA_Pranav_Variant_B.json")
    failure = load_correction("recipes/ASTRA_Pranav_Failure_Injection.json")
    result = evaluate(failure, recipe.constraints)
    assert not result.accepted
    joined = " ".join(result.reasons)
    assert "low_confidence" in joined
    assert "translation_out_of_bounds" in joined
    assert "rotation_out_of_bounds" in joined


def test_gate_targets_match_recipe_and_job():
    recipe = load_recipe("recipes/ASTRA_Pranav_Variant_B.json")
    correction = load_correction("recipes/ASTRA_Pranav_Perception_Correction.json")
    failure = load_correction("recipes/ASTRA_Pranav_Failure_Injection.json")
    assert correction.job_id == failure.job_id == recipe.job_id == "ASTRA_PRANAV_B"
    assert correction.part_id == failure.part_id == "member_B"
