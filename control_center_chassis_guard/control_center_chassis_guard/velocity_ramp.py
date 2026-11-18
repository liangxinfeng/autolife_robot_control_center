"""Slew-rate limited velocity shaping for the control center manual driving pages.

The navigation and mapping pages used to publish the requested velocity as soon
as a key changed: pressing W stepped the command from 0 straight to the slider
speed, releasing it stepped straight back to 0.  On this platform the upper body
is a humanoid carried by a four-wheel mecanum base with a raised centre of mass,
so an acceleration step rocks the body at start and a deceleration step lets the
torso swing forward on stop.

Both pages already publish through a periodic tick (the navigation node's
heartbeat thread at ``manual_cmd_vel_rate_hz``, the mapping page's 20 ms
velocity timer).  This module turns that tick into a slew-rate limiter: the
keyboard only ever sets a *target* velocity, and each tick moves the *commanded*
velocity toward the target by at most ``accel * dt`` -- or ``decel * dt`` when
the magnitude shrinks, the target is zero, or the direction reverses.

That keeps the operator feel intact: the target still changes on the key event
itself, the chassis starts moving on the very next tick, and a release starts
decelerating immediately.  Only the rate of change is bounded, so no input delay
is introduced.  A direction reversal passes through zero because reversing
counts as deceleration until the command crosses the sign boundary.

Acceleration and deceleration are separate per axis, so start-up and stop can be
tuned independently for the linear and the angular axis.  The operative limits
live in ``chassis_teleop_smoothing.yaml`` next to this module, shared by both
pages, so one file keeps the two pages consistent; the ``DEFAULT_*`` constants
below are only the fallback used when that file cannot be read, and they mirror
the last validated tuning so a missing file does not silently change the feel.

``VelocityRampDiagnostics`` is an opt-in trace that records one line per change
of the commanded velocity -- enough to prove the ramp exists, silent while the
command is steady (``CONTROL_CENTER_VELDIAG=1``).
"""

import json
import math
import os
import re
import tempfile
import time
from pathlib import Path

try:  # PyYAML ships with the ROS 2 desktop install used by the control center.
    import yaml
except ImportError:  # pragma: no cover - falls back to the built-in limits
    yaml = None


DEFAULT_LINEAR_ACCEL = 0.5   # m/s^2
DEFAULT_LINEAR_DECEL = 0.5   # m/s^2
DEFAULT_ANGULAR_ACCEL = 1.3  # rad/s^2
DEFAULT_ANGULAR_DECEL = 1.0  # rad/s^2
DEFAULT_UPDATE_HZ = 50.0
DEFAULT_CONFIG_NAME = 'chassis_teleop_smoothing.yaml'

# A stalled UI thread must not turn a single tick into a full-speed jump, so the
# time credited to one step is capped.  0.1 s at the linear limit is 0.05 m/s.
MAX_STEP_DT = 0.1

# Below this an axis counts as exactly zero, so ``idle`` and the lease hand-back
# are decided on exact zeroes rather than on float noise.
AXIS_EPSILON = 1e-9

_CONFIG_FIELDS = (
    'linear_accel',
    'linear_decel',
    'angular_accel',
    'angular_decel',
    'update_hz',
)


class SmoothingConfig:
    """Acceleration/deceleration limits shared by both manual driving pages."""

    __slots__ = _CONFIG_FIELDS + ('source',)

    def __init__(self, linear_accel=DEFAULT_LINEAR_ACCEL,
                 linear_decel=DEFAULT_LINEAR_DECEL,
                 angular_accel=DEFAULT_ANGULAR_ACCEL,
                 angular_decel=DEFAULT_ANGULAR_DECEL,
                 update_hz=DEFAULT_UPDATE_HZ, source='defaults'):
        self.linear_accel = _positive('linear_accel', linear_accel)
        self.linear_decel = _positive('linear_decel', linear_decel)
        self.angular_accel = _positive('angular_accel', angular_accel)
        self.angular_decel = _positive('angular_decel', angular_decel)
        self.update_hz = _positive('update_hz', update_hz)
        self.source = source

    @property
    def period(self):
        """Nominal seconds between two ramp ticks."""
        return 1.0 / self.update_hz

    def as_dict(self):
        return {field: getattr(self, field) for field in _CONFIG_FIELDS}


def _positive(name, value):
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError('%s must be a positive number, got %r' % (name, value))
    return number


def default_config_path():
    """The single shared limits file, shipped next to this module."""
    return Path(__file__).resolve().with_name(DEFAULT_CONFIG_NAME)


def load_smoothing_config(path=None):
    """Read the shared limits, falling back to the built-in defaults.

    A missing or malformed file is not an error: manual driving must stay
    available, so the defaults are used instead.
    """
    target = default_config_path() if path is None else Path(path)
    values = _read_config_values(target)
    try:
        return SmoothingConfig(source=str(target), **values)
    except ValueError:
        return SmoothingConfig(source='%s (invalid, defaults used)' % target)


def _read_config_values(target):
    if yaml is None:
        return {}
    try:
        with open(target, 'r', encoding='utf-8') as handle:
            loaded = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError):
        return {}
    if not isinstance(loaded, dict):
        return {}
    section = loaded.get('manual_teleop_smoothing', loaded)
    if not isinstance(section, dict):
        return {}
    return {key: section[key] for key in _CONFIG_FIELDS if key in section}


def _sanitize_tag(tag):
    """Reduce a caller-supplied tag to characters that cannot alter a path."""
    if not tag:
        return ''
    cleaned = re.sub(r'[^A-Za-z0-9_-]', '', str(tag))
    return ('_' + cleaned) if cleaned else ''


class VelocityRamp:
    """Slew-rate limiter turning a requested velocity into a commanded velocity.

    ``set_target`` records what the keyboard asks for; ``step`` advances the
    command toward it and returns the value that should be published.  Callers
    must serialize access (the navigation node holds ``_velocity_lock``).
    """

    def __init__(self, config=None, linear_accel=None, linear_decel=None,
                 angular_accel=None, angular_decel=None, update_hz=None,
                 clock=None, diagnostics=None):
        limits = load_smoothing_config() if config is None else config
        self._linear_accel = _positive(
            'linear_accel',
            limits.linear_accel if linear_accel is None else linear_accel)
        self._linear_decel = _positive(
            'linear_decel',
            limits.linear_decel if linear_decel is None else linear_decel)
        self._angular_accel = _positive(
            'angular_accel',
            limits.angular_accel if angular_accel is None else angular_accel)
        self._angular_decel = _positive(
            'angular_decel',
            limits.angular_decel if angular_decel is None else angular_decel)
        self._update_hz = _positive(
            'update_hz', limits.update_hz if update_hz is None else update_hz)
        self.config = limits
        self._clock = time.monotonic if clock is None else clock
        self._diagnostics = diagnostics
        self._target = [0.0, 0.0, 0.0]
        self._current = [0.0, 0.0, 0.0]
        self._last_step = None

    @property
    def target(self):
        return tuple(self._target)

    @property
    def current(self):
        return tuple(self._current)

    @property
    def idle(self):
        """True when the target and the commanded velocity are both zero."""
        return all(abs(value) <= AXIS_EPSILON for value in self._target) and \
            all(abs(value) <= AXIS_EPSILON for value in self._current)

    @property
    def update_hz(self):
        return self._update_hz

    def set_target(self, linear_x, linear_y, angular_z):
        """Record the requested velocity; the next :meth:`step` starts moving.

        The target changes immediately, so a key press is never delayed -- only
        the commanded velocity is rate limited.
        """
        self._target = [float(linear_x), float(linear_y), float(angular_z)]
        return self.target

    def step(self, now=None):
        """Advance the commanded velocity toward the target and return it."""
        current_time = self._now(now)
        dt = self._tick_dt(current_time)
        self._last_step = current_time
        limits = (
            (self._linear_accel, self._linear_decel),
            (self._linear_accel, self._linear_decel),
            (self._angular_accel, self._angular_decel),
        )
        for index, (accel, decel) in enumerate(limits):
            self._current[index] = _advance(
                self._target[index], self._current[index], accel, decel, dt)
        self._trace(current_time)
        return self.current

    def reset(self):
        """Snap target and command to zero at once (safety paths only).

        Normal key releases must use :meth:`set_target` with zeroes so the stop
        stays ramped; this is only for X key, focus loss, page switch, window
        close, lost lease and shutdown, which keep their instant-stop meaning.
        """
        was_moving = not self.idle
        self._target = [0.0, 0.0, 0.0]
        self._current = [0.0, 0.0, 0.0]
        self._last_step = None
        if was_moving:
            self._trace(self._now(None))
        return was_moving

    def _tick_dt(self, current_time):
        if self._last_step is None:
            # First step of a run: assume one nominal tick so a key press moves
            # the chassis on this event instead of waiting for the next one.
            return 1.0 / self._update_hz
        return min(max(current_time - self._last_step, 0.0), MAX_STEP_DT)

    def _trace(self, current_time):
        if self._diagnostics is not None:
            self._diagnostics.record(
                current_time, self._target, self._current)

    def _now(self, now):
        return self._clock() if now is None else float(now)


def _advance(target, current, accel, decel, dt):
    """Move ``current`` toward ``target`` by at most one limit-scaled step."""
    error = target - current
    if abs(error) <= AXIS_EPSILON:
        return target
    speeding_up = (
        abs(target) > abs(current)
        and (abs(current) <= AXIS_EPSILON or target * current > 0.0)
    )
    maximum = (accel if speeding_up else decel) * dt
    if abs(error) <= maximum:
        return target
    return current + math.copysign(maximum, error)


class VelocityRampDiagnostics:
    """Opt-in JSON-lines trace of target versus commanded velocity.

    Disabled unless ``CONTROL_CENTER_VELDIAG`` is set to a truthy value.  The
    trace always lands in the system temp directory under a name built from a
    sanitized ``tag`` and the pid; no caller- or environment-supplied path is
    ever opened, so the trace cannot be pointed outside that directory.  One
    line per change of the commanded velocity, so a ramp shows every
    intermediate step while a steady command stays silent.  Every write is
    best-effort and must never break motion handling.
    """

    def __init__(self, tag=None, enabled=None, clock=None):
        if enabled is None:
            enabled = os.environ.get('CONTROL_CENTER_VELDIAG', '') not in (
                '', '0', 'false', 'False', 'no')
        self._clock = time.monotonic if clock is None else clock
        self._file = None
        self.path = None
        self._last = None
        if not enabled:
            return
        name = 'control_center_veldiag%s_%d.log' % (
            _sanitize_tag(tag), os.getpid())
        target = os.path.join(tempfile.gettempdir(), name)
        try:
            self._file = open(target, 'a', encoding='utf-8')
        except OSError:
            self._file = None
        else:
            self.path = target

    @property
    def enabled(self):
        return self._file is not None

    def record(self, timestamp, target, current):
        if self._file is None:
            return
        values = [round(float(value), 6) for value in target] + \
            [round(float(value), 6) for value in current]
        if values == self._last:
            return
        self._last = values
        payload = {
            't': round(float(timestamp), 6),
            'target_linear_x': values[0],
            'target_linear_y': values[1],
            'target_angular_z': values[2],
            'output_linear_x': values[3],
            'output_linear_y': values[4],
            'output_angular_z': values[5],
        }
        try:
            self._file.write(json.dumps(payload) + '\n')
            self._file.flush()
        except OSError:
            pass

    def close(self):
        if self._file is not None:
            try:
                self._file.close()
            except OSError:
                pass
            self._file = None
