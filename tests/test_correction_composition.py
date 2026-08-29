"""R11 test 3: correction composition - applying a correction is idempotent
when re-derived from nominal (never double-applied across re-plans, R8)."""
import numpy as np

from astra_core.geometry.pose import Pose
from astra_core.perception.correction import corrected_pose_from_nominal
from astra_core.recipe.models import PerceptionCorrection
from astra_core.world.part_state import PartState


def _correction(**overrides):
    defaults = dict(
        job_id="J",
        part_id="member_B",
        confidence=0.93,
        delta_translation_m=np.array([0.012, -0.007, 0.004]),
        delta_rpy_deg=np.array([0.0, 0.0, 3.5]),
    )
    defaults.update(overrides)
    return PerceptionCorrection(**defaults)


def test_composing_from_nominal_twice_gives_identical_pose():
    nominal = Pose.from_xyz_rpy([0.38, 0.32, 0.1], [0, 0, 1.45])
    correction = _correction()

    first = corrected_pose_from_nominal(nominal, correction)
    second = corrected_pose_from_nominal(nominal, correction)  # re-derived from nominal again

    assert first == second


def test_part_state_apply_correction_does_not_stack():
    nominal = Pose.from_xyz_rpy([0.38, 0.32, 0.1], [0, 0, 1.45])
    state = PartState(part_id="member_B", nominal_pose=nominal)
    correction = _correction()

    once = corrected_pose_from_nominal(state.nominal_pose, correction)
    state.apply_correction(once)
    first_effective = state.effective_pose

    # Simulate a second re-plan pass: correction must be re-derived from the
    # PART'S NOMINAL pose, not from state.effective_pose (which is already
    # corrected) - the orchestrator always calls corrected_pose_from_nominal
    # with part.source_pose, never with world.pose_of(part_id).
    twice = corrected_pose_from_nominal(state.nominal_pose, correction)
    state.apply_correction(twice)
    second_effective = state.effective_pose

    assert first_effective == second_effective
    # Guard against the double-application bug directly: composing from the
    # ALREADY-corrected pose would move the part further than composing from
    # nominal - assert that particular mistake would in fact differ.
    wrong_double_apply = corrected_pose_from_nominal(first_effective, correction)
    assert not np.allclose(wrong_double_apply.xyz, first_effective.xyz, atol=1e-9)
