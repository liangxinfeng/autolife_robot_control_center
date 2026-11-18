"""Unit tests for the manual key-state debounce filter.

The regression these tests encode is the acceptance finding from 2026-09-10:
while a WASD key was held, X11/NoMachine input delivered real
release+press pairs (``isAutoRepeat() == False``), the pressed set emptied for
one frame and the page published a single-frame zero velocity ~19-21% of the
time.  The filter must keep the key effectively held across such pairs.
"""

import json

import pytest

from control_center_chassis_guard.manual_key_filter import (
    KeyEventDiagnostics,
    ManualKeyFilter,
)


class FakeClock:
    def __init__(self, start=1000.0):
        self.value = start

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


KEY_W = 87
KEY_A = 65


class TestDebounce:
    def test_press_is_immediate(self):
        keys = ManualKeyFilter(release_delay_sec=0.03, clock=FakeClock())
        assert keys.press(KEY_W) is True
        assert keys.pressed_keys() == frozenset({KEY_W})

    def test_release_is_deferred_not_immediate(self):
        clock = FakeClock()
        keys = ManualKeyFilter(release_delay_sec=0.03, clock=clock)
        keys.press(KEY_W)
        assert keys.release(KEY_W) is True
        # Still logically held while the release is pending ...
        assert keys.pressed_keys() == frozenset({KEY_W})
        assert keys.pending_count == 1
        # ... until the delay elapses.
        clock.advance(0.029)
        assert keys.expire() is False
        assert keys.pressed_keys() == frozenset({KEY_W})
        clock.advance(0.002)
        assert keys.expire() is True
        assert keys.pressed_keys() == frozenset()

    def test_release_press_pair_within_window_keeps_key_held(self):
        clock = FakeClock()
        keys = ManualKeyFilter(release_delay_sec=0.03, clock=clock)
        keys.press(KEY_W)
        keys.release(KEY_W)
        clock.advance(0.0005)  # X11 pair gap is sub-millisecond in the field
        assert keys.press(KEY_W) is True  # cancels the pending release
        assert keys.pending_count == 0
        clock.advance(1.0)
        assert keys.expire() is False
        assert keys.pressed_keys() == frozenset({KEY_W})

    def test_release_of_unpressed_key_is_ignored(self):
        keys = ManualKeyFilter(release_delay_sec=0.03, clock=FakeClock())
        assert keys.release(KEY_W) is False
        assert keys.pending_count == 0

    def test_repeated_release_does_not_extend_forever(self):
        clock = FakeClock()
        keys = ManualKeyFilter(release_delay_sec=0.03, clock=clock)
        keys.press(KEY_W)
        keys.release(KEY_W)
        clock.advance(0.02)
        keys.release(KEY_W)  # duplicate release event
        clock.advance(0.011)  # 0.031 s after first scheduled release
        assert keys.expire(now=1000.031) is True
        assert keys.pressed_keys() == frozenset()

    def test_zero_delay_releases_immediately(self):
        keys = ManualKeyFilter(release_delay_sec=0.0, clock=FakeClock())
        keys.press(KEY_W)
        assert keys.release(KEY_W) is True
        assert keys.pressed_keys() == frozenset()
        assert keys.next_deadline() is None

    def test_keys_are_independent(self):
        clock = FakeClock()
        keys = ManualKeyFilter(release_delay_sec=0.03, clock=clock)
        keys.press(KEY_W)
        keys.press(KEY_A)
        keys.release(KEY_W)
        clock.advance(0.04)
        assert keys.expire() is True
        assert keys.pressed_keys() == frozenset({KEY_A})

    def test_clear_drops_everything_immediately(self):
        clock = FakeClock()
        keys = ManualKeyFilter(release_delay_sec=0.03, clock=clock)
        keys.press(KEY_W)
        keys.release(KEY_W)
        assert keys.clear() is True
        assert keys.pressed_keys() == frozenset()
        assert keys.pending_count == 0
        assert keys.next_deadline() is None

    def test_next_deadline_tracks_earliest_release(self):
        clock = FakeClock()
        keys = ManualKeyFilter(release_delay_sec=0.03, clock=clock)
        keys.press(KEY_W)
        keys.press(KEY_A)
        keys.release(KEY_A)
        clock.advance(0.01)
        keys.release(KEY_W)
        assert keys.next_deadline() == pytest.approx(clock() + 0.02)

    def test_negative_delay_rejected(self):
        with pytest.raises(ValueError):
            ManualKeyFilter(release_delay_sec=-0.01)


class TestFieldRegression:
    """The exact field waveform: sustained hold with X11 auto-repeat pairs."""

    def test_ten_seconds_of_pairs_never_empties_the_pressed_set(self):
        clock = FakeClock()
        keys = ManualKeyFilter(release_delay_sec=0.03, clock=clock)
        keys.press(KEY_W)
        # X11 auto-repeat is configured at 33/s (~30 ms) on 306; NoMachine
        # delivered each repeat as release+press with isAutoRepeat()==False.
        for _ in range(334):  # ~10 s
            keys.release(KEY_W)
            keys.press(KEY_W)
            clock.advance(0.030)
            assert keys.expire() is False
            assert keys.pressed_keys() == frozenset({KEY_W}), (
                'pressed set emptied mid-hold; the page would publish a zero frame')

    def test_real_release_after_hold_still_stops(self):
        clock = FakeClock()
        keys = ManualKeyFilter(release_delay_sec=0.03, clock=clock)
        keys.press(KEY_W)
        for _ in range(20):
            keys.release(KEY_W)
            keys.press(KEY_W)
            clock.advance(0.030)
        keys.release(KEY_W)  # genuine release, no re-press
        clock.advance(0.031)
        assert keys.expire() is True
        assert keys.pressed_keys() == frozenset()


class TestDiagnostics:
    def test_disabled_by_default(self, monkeypatch):
        monkeypatch.delenv('CONTROL_CENTER_KEYDIAG', raising=False)
        diag = KeyEventDiagnostics()
        assert diag.enabled is False
        diag.log('press', key=KEY_W)  # must be a harmless no-op
        diag.close()

    def test_env_flag_enables_and_writes_json_lines(self, tmp_path, monkeypatch):
        target = tmp_path / 'keydiag.log'
        monkeypatch.setenv('CONTROL_CENTER_KEYDIAG', '1')
        monkeypatch.setenv('CONTROL_CENTER_KEYDIAG_PATH', str(target))
        diag = KeyEventDiagnostics()
        assert diag.enabled is True
        diag.log('press', key=KEY_W, pending=0)
        diag.log('release', key=KEY_W, pending=1)
        diag.close()
        lines = [json.loads(line) for line in
                 target.read_text(encoding='utf-8').splitlines()]
        assert [entry['kind'] for entry in lines] == ['press', 'release']
        assert lines[0]['key'] == KEY_W

    def test_unwritable_path_is_inert(self, monkeypatch):
        monkeypatch.setenv('CONTROL_CENTER_KEYDIAG', '1')
        monkeypatch.setenv('CONTROL_CENTER_KEYDIAG_PATH',
                           '/nonexistent-dir/zcode/keydiag.log')
        diag = KeyEventDiagnostics()
        assert diag.enabled is False
        diag.log('press', key=KEY_W)  # must not raise
        diag.close()
