"""Unit tests for the manual teleop slew-rate limiter.

Covers the acceptance cases for the 2026-09-15 teleop smoothing task: start-up
from rest is a bounded ramp rather than a step, a normal release decelerates to
exactly zero, a direction reversal crosses zero instead of flipping sign, quick
taps do not run away, long holds still reach the requested speed, and the four
limits are applied independently per axis.
"""

import json
import math
import tempfile
from pathlib import Path

import pytest

from control_center_chassis_guard.velocity_ramp import (
    DEFAULT_ANGULAR_ACCEL,
    DEFAULT_ANGULAR_DECEL,
    DEFAULT_LINEAR_ACCEL,
    DEFAULT_LINEAR_DECEL,
    DEFAULT_UPDATE_HZ,
    MAX_STEP_DT,
    SmoothingConfig,
    VelocityRamp,
    VelocityRampDiagnostics,
    default_config_path,
    load_smoothing_config,
)

TICK = 1.0 / 50.0
LINEAR_ACCEL = 0.5
LINEAR_DECEL = 0.5
ANGULAR_ACCEL = 1.0
ANGULAR_DECEL = 1.0


class FakeClock:
    def __init__(self, start=1000.0):
        self.value = start

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


def make_ramp(clock=None, diagnostics=None, **overrides):
    limits = {
        'linear_accel': LINEAR_ACCEL,
        'linear_decel': LINEAR_DECEL,
        'angular_accel': ANGULAR_ACCEL,
        'angular_decel': ANGULAR_DECEL,
        'update_hz': 1.0 / TICK,
    }
    limits.update(overrides)
    return VelocityRamp(
        config=SmoothingConfig(**limits),
        clock=clock if clock is not None else FakeClock(),
        diagnostics=diagnostics,
    )


def run_ticks(ramp, clock, count, tick=TICK):
    """Advance the clock and step the ramp, returning every output value."""
    outputs = []
    for _ in range(count):
        clock.advance(tick)
        outputs.append(ramp.step())
    return outputs


def test_shipped_config_is_read_from_the_yaml_next_to_the_module():
    config = load_smoothing_config()
    assert config.source == str(default_config_path())
    assert math.isclose(config.period, 1.0 / config.update_hz, abs_tol=1e-12)


def test_shipped_limits_stay_in_a_sane_tuning_range():
    """A typo in the tuning file must not silently break manual driving.

    The exact values are the operator's tuning surface and are expected to
    change after real-robot tests, so this only guards the range.
    """
    config = load_smoothing_config()
    assert 0.1 <= config.linear_accel <= 2.0
    assert 0.1 <= config.linear_decel <= 2.0
    assert 0.1 <= config.angular_accel <= 2.0
    assert 0.1 <= config.angular_decel <= 2.0
    assert 20.0 <= config.update_hz <= 100.0


def test_missing_config_falls_back_to_builtin_limits():
    config = load_smoothing_config(Path(tempfile.gettempdir()) / 'no_such.yaml')
    assert config.linear_accel == DEFAULT_LINEAR_ACCEL
    assert config.update_hz == DEFAULT_UPDATE_HZ


def test_an_invalid_tuning_value_falls_back_to_builtin_limits(tmp_path):
    path = tmp_path / 'limits.yaml'
    path.write_text(
        'manual_teleop_smoothing:\n  angular_accel: 0\n', encoding='utf-8')
    config = load_smoothing_config(path)
    assert config.angular_accel == DEFAULT_ANGULAR_ACCEL
    assert config.linear_accel == DEFAULT_LINEAR_ACCEL


def test_independent_limits_are_read_from_a_config_file(tmp_path):
    path = tmp_path / 'limits.yaml'
    path.write_text(
        'manual_teleop_smoothing:\n'
        '  linear_accel: 0.2\n'
        '  linear_decel: 0.9\n'
        '  angular_accel: 1.5\n'
        '  angular_decel: 0.3\n'
        '  update_hz: 25.0\n',
        encoding='utf-8',
    )
    config = load_smoothing_config(path)
    assert (config.linear_accel, config.linear_decel) == (0.2, 0.9)
    assert (config.angular_accel, config.angular_decel) == (1.5, 0.3)
    assert config.update_hz == 25.0


@pytest.mark.parametrize('value', [0.0, -1.0, float('nan'), float('inf')])
def test_non_positive_limits_are_rejected(value):
    with pytest.raises(ValueError):
        SmoothingConfig(linear_accel=value)


def test_case1_rest_to_forward_is_a_ramp_not_a_step():
    clock = FakeClock()
    ramp = make_ramp(clock)
    ramp.set_target(0.20, 0.0, 0.0)

    outputs = run_ticks(ramp, clock, 20)
    linear = [output[0] for output in outputs]

    assert linear[0] > 0.0, 'the first tick must already produce motion'
    assert linear[0] < 0.20, 'the first tick must not jump to the target'
    assert linear == sorted(linear), 'start-up must be monotonic'
    for previous, current in zip(linear, linear[1:]):
        assert current - previous <= LINEAR_ACCEL * TICK + 1e-9


def test_case2_release_decelerates_to_exactly_zero():
    clock = FakeClock()
    ramp = make_ramp(clock)
    ramp.set_target(0.20, 0.0, 0.0)
    run_ticks(ramp, clock, 40)
    assert ramp.current[0] == pytest.approx(0.20)

    ramp.set_target(0.0, 0.0, 0.0)
    outputs = [output[0] for output in run_ticks(ramp, clock, 40)]

    assert outputs[0] < 0.20, 'release must decelerate on the first tick'
    assert outputs == sorted(outputs, reverse=True), 'stop must be monotonic'
    assert outputs[-1] == 0.0, 'the ramp must land on exactly zero'
    assert ramp.idle


def test_case3_rest_to_rotation_ramps_the_angular_axis():
    clock = FakeClock()
    ramp = make_ramp(clock)
    ramp.set_target(0.0, 0.0, 0.80)

    outputs = [output[2] for output in run_ticks(ramp, clock, 10)]

    assert outputs[0] > 0.0 and outputs[0] < 0.80
    for previous, current in zip(outputs, outputs[1:]):
        assert current - previous <= ANGULAR_ACCEL * TICK + 1e-9


def test_case4_rotation_release_ramps_the_angular_axis_to_zero():
    clock = FakeClock()
    ramp = make_ramp(clock)
    ramp.set_target(0.0, 0.0, 0.80)
    run_ticks(ramp, clock, 60)

    ramp.set_target(0.0, 0.0, 0.0)
    outputs = [output[2] for output in run_ticks(ramp, clock, 60)]

    assert outputs[-1] == 0.0
    assert outputs == sorted(outputs, reverse=True)


def test_case5_forward_to_reverse_crosses_zero_without_flipping():
    clock = FakeClock()
    ramp = make_ramp(clock)
    ramp.set_target(0.20, 0.0, 0.0)
    run_ticks(ramp, clock, 40)

    ramp.set_target(-0.20, 0.0, 0.0)
    outputs = [output[0] for output in run_ticks(ramp, clock, 80)]

    assert outputs[0] < 0.20, 'reversal must start by slowing down'
    assert min(outputs) == 0.0 or 0.0 in outputs or any(
        output <= 0.0 for output in outputs), 'the command must reach zero'
    # The sign may only change through a zero crossing, never in one step.
    previous = 0.20
    for current in outputs:
        if previous > 0.0 and current < 0.0:
            assert abs(previous) <= LINEAR_DECEL * TICK + 1e-9, (
                'sign flipped without passing through zero')
        previous = current
    assert outputs[-1] == pytest.approx(-0.20)


def test_case5_rotation_reversal_crosses_zero():
    clock = FakeClock()
    ramp = make_ramp(clock)
    ramp.set_target(0.0, 0.0, 0.80)
    run_ticks(ramp, clock, 60)

    ramp.set_target(0.0, 0.0, -0.80)
    outputs = [output[2] for output in run_ticks(ramp, clock, 120)]

    assert outputs[0] < 0.80
    assert outputs[-1] == pytest.approx(-0.80)
    previous = 0.80
    for current in outputs:
        if previous > 0.0 and current < 0.0:
            assert abs(previous) <= ANGULAR_DECEL * TICK + 1e-9
        previous = current


def test_case6_quick_tap_returns_to_zero_without_running_on():
    clock = FakeClock()
    ramp = make_ramp(clock)
    ramp.set_target(0.20, 0.0, 0.0)
    run_ticks(ramp, clock, 2)  # a tap of roughly two ticks
    peak = ramp.current[0]
    assert 0.0 < peak < 0.05, 'a tap must not reach the slider speed'

    ramp.set_target(0.0, 0.0, 0.0)
    outputs = run_ticks(ramp, clock, 20)

    assert outputs[-1][0] == 0.0
    # Peak speed stays proportional to the hold time, not the slider speed.
    assert peak <= LINEAR_ACCEL * 3 * TICK + 1e-9


def test_case7_long_hold_reaches_the_requested_speed():
    clock = FakeClock()
    ramp = make_ramp(clock)
    ramp.set_target(0.20, 0.0, 0.0)

    run_ticks(ramp, clock, 200)

    assert ramp.current == (0.20, 0.0, 0.0)
    assert ramp.target == (0.20, 0.0, 0.0)


def test_case8_lateral_axis_ramps_independently_of_forward():
    clock = FakeClock()
    ramp = make_ramp(clock)
    ramp.set_target(0.20, 0.12, 0.40)

    outputs = run_ticks(ramp, clock, 60)

    assert outputs[-1] == (0.20, 0.12, 0.40)
    for output in outputs[:5]:
        assert output[0] <= LINEAR_ACCEL * 6 * TICK + 1e-9
        assert output[1] <= LINEAR_ACCEL * 6 * TICK + 1e-9
        assert output[2] <= ANGULAR_ACCEL * 6 * TICK + 1e-9


def test_asymmetric_limits_choose_accel_when_speeding_up_and_decel_when_not():
    clock = FakeClock()
    ramp = make_ramp(clock, linear_accel=1.0, linear_decel=0.25)

    ramp.set_target(0.30, 0.0, 0.0)
    clock.advance(TICK)
    assert ramp.step()[0] == pytest.approx(1.0 * TICK)

    clock.advance(TICK)
    assert ramp.step()[0] == pytest.approx(2.0 * TICK)

    # Still speeding up towards a higher target uses the acceleration limit.
    ramp.set_target(0.30, 0.0, 0.0)
    previous = ramp.current[0]
    clock.advance(TICK)
    assert ramp.step()[0] - previous == pytest.approx(1.0 * TICK)

    # Slowing down to a lower same-direction target uses the deceleration limit.
    ramp.set_target(0.01, 0.0, 0.0)
    previous = ramp.current[0]
    clock.advance(TICK)
    assert previous - ramp.step()[0] == pytest.approx(0.25 * TICK)


def test_reverse_speeding_up_uses_the_acceleration_limit():
    clock = FakeClock()
    ramp = make_ramp(clock, linear_accel=1.0, linear_decel=0.25)
    ramp.set_target(-0.30, 0.0, 0.0)

    run_ticks(ramp, clock, 12)  # cross zero and start accelerating backwards
    before = ramp.current[0]
    clock.advance(TICK)
    after = ramp.step()[0]

    if before < 0.0:
        assert before - after == pytest.approx(1.0 * TICK, abs=1e-9)


def test_a_stalled_tick_cannot_produce_a_full_speed_jump():
    clock = FakeClock()
    ramp = make_ramp(clock)
    ramp.set_target(0.20, 0.0, 0.0)

    clock.advance(5.0)  # the UI thread was blocked for five seconds
    output = ramp.step()

    assert output[0] <= LINEAR_ACCEL * MAX_STEP_DT + 1e-9


def test_first_tick_uses_the_nominal_period():
    clock = FakeClock()
    ramp = make_ramp(clock)
    ramp.set_target(0.20, 0.0, 0.0)

    assert ramp.step()[0] == pytest.approx(LINEAR_ACCEL * TICK)


def test_idle_is_false_while_any_axis_is_moving():
    clock = FakeClock()
    ramp = make_ramp(clock)
    assert ramp.idle
    ramp.set_target(0.0, 0.0, 0.10)
    assert not ramp.idle
    run_ticks(ramp, clock, 20)
    assert not ramp.idle
    ramp.set_target(0.0, 0.0, 0.0)
    run_ticks(ramp, clock, 20)
    assert ramp.idle


def test_reset_snaps_everything_to_zero_at_once():
    clock = FakeClock()
    ramp = make_ramp(clock)
    ramp.set_target(0.20, 0.0, 0.40)
    run_ticks(ramp, clock, 10)

    assert ramp.reset() is True

    assert ramp.target == (0.0, 0.0, 0.0)
    assert ramp.current == (0.0, 0.0, 0.0)
    assert ramp.idle
    assert ramp.reset() is False, 'resetting a stopped ramp is a no-op'


def test_zero_target_and_zero_command_are_reported_as_exactly_zero():
    clock = FakeClock()
    ramp = make_ramp(clock)
    ramp.set_target(0.0, 0.0, 0.0)

    assert ramp.step() == (0.0, 0.0, 0.0)
    assert ramp.idle


@pytest.fixture()
def veldiag():
    """Enabled trace with a distinct tag, deleted again after the test."""
    created = []

    def factory(tag):
        diagnostics = VelocityRampDiagnostics(tag=tag, enabled=True)
        assert diagnostics.enabled, 'diagnostics did not open a trace file'
        assert Path(diagnostics.path).parent == Path(tempfile.gettempdir())
        created.append(Path(diagnostics.path))
        return diagnostics

    yield factory
    for path in created:
        path.unlink(missing_ok=True)


class TestDiagnostics:
    def _lines_since(self, diagnostics, offset):
        with open(diagnostics.path, 'r', encoding='utf-8') as handle:
            handle.seek(offset)
            return [json.loads(line) for line in handle if line.strip()]

    def _offset(self, diagnostics):
        path = Path(diagnostics.path)
        return path.stat().st_size if path.exists() else 0

    def test_disabled_by_default(self, monkeypatch):
        monkeypatch.delenv('CONTROL_CENTER_VELDIAG', raising=False)
        diagnostics = VelocityRampDiagnostics(tag='unit_default')
        assert not diagnostics.enabled
        assert diagnostics.path is None

    def test_records_target_and_output_per_change(self, veldiag):
        diagnostics = veldiag('unit_shapes')
        clock = FakeClock()
        ramp = make_ramp(clock, diagnostics=diagnostics)
        offset = self._offset(diagnostics)

        ramp.set_target(0.20, 0.0, 0.0)
        for _ in range(3):
            clock.advance(TICK)
            ramp.step()
        ramp.set_target(0.0, 0.0, 0.0)
        for _ in range(3):
            clock.advance(TICK)
            ramp.step()
        diagnostics.close()

        lines = self._lines_since(diagnostics, offset)
        assert len(lines) >= 6, 'the ramp must record every intermediate step'
        for line in lines:
            assert set(line) == {
                't', 'target_linear_x', 'target_linear_y', 'target_angular_z',
                'output_linear_x', 'output_linear_y', 'output_angular_z',
            }
        assert lines[0]['target_linear_x'] == 0.20
        assert lines[0]['output_linear_x'] == pytest.approx(LINEAR_ACCEL * TICK)
        assert lines[-1]['output_linear_x'] == 0.0

    def test_a_steady_command_is_not_logged_repeatedly(self, veldiag):
        diagnostics = veldiag('unit_steady')
        clock = FakeClock()
        ramp = make_ramp(clock, diagnostics=diagnostics)
        offset = self._offset(diagnostics)

        ramp.set_target(0.20, 0.0, 0.0)
        run_ticks(ramp, clock, 200)
        diagnostics.close()

        lines = self._lines_since(diagnostics, offset)
        # 0.20 m/s at 0.5 m/s^2 needs 20 ticks to be reached; after that the
        # command is unchanged and must not keep writing.
        assert len(lines) == 20

    def test_a_hostile_tag_cannot_escape_the_temp_directory(self, veldiag):
        diagnostics = veldiag('../../../etc/passwd')
        assert Path(diagnostics.path).parent == Path(tempfile.gettempdir())
        assert '..' not in str(diagnostics.path)
        assert '/' not in Path(diagnostics.path).name
        diagnostics.close()
