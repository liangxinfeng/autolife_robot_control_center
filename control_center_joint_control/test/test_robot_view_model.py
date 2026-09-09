from pathlib import Path

import numpy as np

from control_center_joint_control.robot_view import (
    RobotModel,
    _load_binary_stl,
    _rotation_guide_geometry,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PACKAGE_ROOT / 'urdf' / 'robot_v2_2.urdf'


def test_urdf_model_has_complete_visual_and_joint_tree():
    model = RobotModel(URDF_PATH)
    transforms = model.link_transforms({})
    assert len(transforms) == 47
    assert len(model.visuals) == 43
    assert all(visual.mesh_path.exists() for visual in model.visuals)


def test_joint_motion_changes_child_link_transform():
    model = RobotModel(URDF_PATH)
    neutral = model.link_transforms({})
    moved = model.link_transforms({'Joint_Left_Elbow': 30.0})
    child = next(
        joint.child
        for joints in model.children.values()
        for joint in joints
        if joint.name == 'Joint_Left_Elbow'
    )
    assert not np.allclose(neutral[child], moved[child])


def test_binary_stl_loader_returns_matching_vertices_and_normals():
    model = RobotModel(URDF_PATH)
    vertices, normals = _load_binary_stl(model.visuals[0].mesh_path)
    assert vertices.ndim == 2 and vertices.shape[1] == 3
    assert normals.shape == vertices.shape
    assert len(vertices) > 0


def test_visual_links_resolve_to_safe_model_adjustment_joints():
    model = RobotModel(URDF_PATH)
    selectable = [
        model.selectable_joint_for_link(visual.link)
        for visual in model.visuals
    ]
    assert 'Joint_Left_Elbow' in selectable
    assert 'Joint_Right_Elbow' in selectable
    assert 'Joint_Waist_Pitch' in selectable
    assert 'Joint_Left_Gripper' not in selectable
    assert 'Joint_Right_Gripper' not in selectable


def test_selected_joint_world_axis_is_normalized_and_at_joint_origin():
    model = RobotModel(URDF_PATH)
    origin, axis = model.joint_world_axis('Joint_Left_Elbow', {})
    assert origin.shape == (3,)
    assert np.isclose(np.linalg.norm(axis), 1.0)
    assert np.all(np.isfinite(origin))


def test_rotation_guide_arrows_follow_positive_and_negative_axis_direction():
    geometry = _rotation_guide_geometry(
        np.zeros(3),
        np.array((0.0, 0.0, 1.0)),
    )
    positive = geometry['positive_arc']
    negative = geometry['negative_arc']
    assert np.dot(np.cross(positive[0], positive[1]), (0.0, 0.0, 1.0)) > 0
    assert np.dot(np.cross(negative[0], negative[1]), (0.0, 0.0, 1.0)) < 0
    assert len(geometry['positive_arrowheads']) == 2
    assert len(geometry['negative_arrowheads']) == 2
