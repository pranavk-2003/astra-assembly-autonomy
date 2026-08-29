"""R11 test 2: transform/pose math - approach-offset derivation, rpy<->quat
round-trip, and the degrees/radians unit boundary."""
import numpy as np
import pytest

from astra_core.geometry.approach import derive_approach_pose, derive_grasp_pose
from astra_core.geometry.pose import Pose
from astra_core.geometry.transforms import (
    rotation_delta_magnitude_deg,
    translation_delta_m,
)


def test_rpy_round_trip():
    rpy_in = np.array([0.1, -0.2, 1.5708])
    pose = Pose.from_xyz_rpy([1, 2, 3], rpy_in)
    rpy_out = pose.to_rpy()
    assert np.allclose(rpy_in, rpy_out, atol=1e-6)


def test_derive_grasp_pose_applies_offset_in_part_frame():
    part_pose = Pose.from_xyz_rpy([0.58, -0.02, 0.11], [0, 0, 0])
    grasp = derive_grasp_pose(part_pose, [0.0, 0.0, 0.025])
    assert np.allclose(grasp.xyz, [0.58, -0.02, 0.135])


def test_derive_grasp_pose_offset_rotates_with_part_orientation():
    # part yawed 90deg: an offset along local +x should land along world +y
    part_pose = Pose.from_xyz_rpy([0, 0, 0], [0, 0, np.pi / 2])
    grasp = derive_grasp_pose(part_pose, [0.1, 0.0, 0.0])
    assert np.allclose(grasp.xyz, [0.0, 0.1, 0.0], atol=1e-9)


@pytest.mark.parametrize("distance", [0.1, 0.12, 0.2])
def test_derive_approach_pose_matches_formula(distance):
    target = Pose.from_xyz_rpy([0.5, 0.1, 0.2], [0, 0, 0])
    vector = [0.0, 0.0, -1.0]
    standoff = derive_approach_pose(target, vector, distance)
    expected = target.xyz - np.array(vector) * distance
    assert np.allclose(standoff.xyz, expected)
    assert standoff == Pose(xyz=standoff.xyz, quat_xyzw=target.quat_xyzw)


def test_derive_approach_pose_rejects_zero_vector():
    target = Pose.identity()
    with pytest.raises(ValueError):
        derive_approach_pose(target, [0, 0, 0], 0.1)


def test_translation_and_rotation_delta_measure_correctly():
    a = Pose.from_xyz_rpy([0, 0, 0], [0, 0, 0])
    b = Pose.from_xyz_rpy([0.012, -0.007, 0.004], np.radians([0, 0, 3.5]))
    delta_t = translation_delta_m(a, b)
    assert np.allclose(np.linalg.norm(delta_t), np.linalg.norm([0.012, -0.007, 0.004]))
    assert abs(rotation_delta_magnitude_deg(a, b) - 3.5) < 1e-6


def test_degrees_vs_radians_boundary_not_confused():
    # A 3.5 DEGREE correction must not be mistaken for 3.5 RADIANS anywhere
    # in the pipeline - this is the assessment spec's flagged "easiest way to fail".
    from astra_core.geometry.transforms import compose_correction

    nominal = Pose.identity()
    corrected = compose_correction(nominal, [0, 0, 0], [0, 0, 3.5])
    rpy_out_deg = np.degrees(corrected.to_rpy())
    assert abs(rpy_out_deg[2] - 3.5) < 1e-6
    assert abs(rpy_out_deg[2] - np.degrees(3.5)) > 1.0  # sanity: not the radian value
