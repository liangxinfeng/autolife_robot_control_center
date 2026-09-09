from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_control_center_defaults_to_regular_nav2_for_every_goal():
    manager = (
        PACKAGE_ROOT / 'scripts' / 'waypoint_manager_node.py'
    ).read_text(encoding='utf-8')
    launch = (
        PACKAGE_ROOT / 'launch' / 'bringup_all.launch.py'
    ).read_text(encoding='utf-8')

    assert "declare_parameter('short_distance_direct_start_enabled', False)" in manager
    assert "'short_distance_direct_start_enabled',\n        default_value='false'" in launch
    assert "if not force_normal and self._short_direct_start_enabled:" in manager
    assert "self._far_navigation_auto_refine_enabled" in manager
