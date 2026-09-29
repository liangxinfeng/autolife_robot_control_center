import math
from pathlib import Path
import pytest
import yaml
from short_distance_servo import corridor_is_clear


def clear(obstacle, yaw=0.0, target=None, extension=0.25):
    resolution = 0.02
    size = 250
    data = [0] * (size * size)
    x, y = obstacle
    data[int((y + 2.5) / resolution) * size + int((x + 2.5) / resolution)] = 254
    return corridor_is_clear(
        data, size, size, resolution, (-2.5, -2.5, 0),
        (0, 0, yaw), target or (0, 0, yaw),
        half_length=0.25, half_width=0.25, front_extension=extension,
    )


@pytest.mark.parametrize('point,expected', [
    ((0.43, 0.01), False), ((0.55, 0.01), True),
    ((-0.35, 0.01), True), ((-0.21, 0.01), False),
    ((0.01, 0.23), False), ((0.01, 0.29), True),
    ((0.01, -0.23), False), ((0.01, -0.29), True),
])
def test_front_extended_width_50_rear_unchanged(point, expected):
    assert clear(point) == expected


def test_extension_follows_robot_orientation():
    assert not clear((0.01, 0.43), yaw=math.pi / 2)
    assert clear((0.01, -0.43), yaw=math.pi / 2)


def test_turn_checks_front_sweep_not_only_endpoints():
    point = (0.33, 0.33)
    assert clear(point)
    assert clear(point, yaw=math.pi / 2)
    assert not clear(point, target=(0, 0, math.pi / 2))


def test_translation_checks_new_front_edge():
    assert clear((0.71, 0.01))
    assert not clear((0.71, 0.01), target=(0.3, 0, 0))


def test_default_extension_preserves_other_callers():
    assert clear((0.43, 0.01), extension=0.0)


def test_both_costmap_footprints_match():
    root = Path(__file__).resolve().parent
    config = root / 'nav2_params.yaml'
    if not config.exists():
        config = root.parent / 'config' / 'nav2_params.yaml'
    config = yaml.safe_load(config.read_text())
    expected = [[0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25]]
    for name in ('local_costmap', 'global_costmap'):
        params = config[name][name]['ros__parameters']
        assert yaml.safe_load(params['footprint']) == expected
        assert params['footprint_padding'] == 0.025
        assert 0.50 + 2 * params['footprint_padding'] == pytest.approx(0.55)
