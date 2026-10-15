"""Hold/release key-state debounce for the control center WASD pages.

Both manual driving pages keep a set of currently pressed keys and publish
whatever that set contains.  On X11 (including the NoMachine remote input path
used on 300) a held key can arrive as a real ``KeyRelease`` immediately
followed by a ``KeyPress`` of the same key, with ``isAutoRepeat() == False`` on
both events.  The pressed set then momentarily becomes empty and the page
publishes a single-frame zero velocity, which the chassis applies as a stutter;
it also made the chassis ownership lease release and re-acquire hundreds of
times per session.

``ManualKeyFilter`` defers a release by ``release_delay_sec`` and cancels the
pending release when the same key is pressed again inside that window, so such
pairs no longer interrupt a held command.  A genuine release is applied after
the short delay; every safety path (X key, focus-out, page switch, window
close, explicit stop) must call :meth:`clear` for an immediate stop.

``KeyEventDiagnostics`` is an opt-in, inert-by-default trace used to confirm the
input jitter mechanism on the robot (``CONTROL_CENTER_KEYDIAG=1``).
"""

import json
import os
import time


DEFAULT_RELEASE_DELAY_SEC = 0.03


class ManualKeyFilter:
    """Pressed-set with a short, cancellable release delay."""

    def __init__(self, release_delay_sec=DEFAULT_RELEASE_DELAY_SEC, clock=None):
        if release_delay_sec < 0:
            raise ValueError('release_delay_sec must not be negative')
        self._release_delay = float(release_delay_sec)
        self._clock = time.monotonic if clock is None else clock
        self._pressed = set()
        self._pending = {}  # key -> monotonic deadline

    @property
    def release_delay_sec(self):
        return self._release_delay

    @property
    def pending_count(self):
        return len(self._pending)

    def pressed_keys(self):
        return frozenset(self._pressed)

    def is_pressed(self, key):
        return key in self._pressed

    def press(self, key, now=None):
        """Mark ``key`` pressed and cancel any pending release.

        Returns True when the effective pressed set changed.
        """
        changed = key not in self._pressed
        if self._pending.pop(key, None) is not None:
            changed = True
        self._pressed.add(key)
        return changed

    def release(self, key, now=None):
        """Schedule ``key`` for release after the debounce delay.

        Returns True when the call was meaningful (key was pressed or already
        pending).  The key stays in :meth:`pressed_keys` until :meth:`expire`
        applies the release, so a same-key ``press`` can still cancel it.  A
        duplicate release never extends the deadline: repeats cannot postpone
        a real stop indefinitely.
        """
        if key not in self._pressed and key not in self._pending:
            return False
        if self._release_delay <= 0.0:
            return self.immediate_release(key)
        if key not in self._pending:
            self._pending[key] = self._now(now) + self._release_delay
        return True

    def immediate_release(self, key, now=None):
        """Drop ``key`` right now, ignoring the debounce delay."""
        changed = key in self._pressed
        self._pressed.discard(key)
        self._pending.pop(key, None)
        return changed

    def expire(self, now=None):
        """Apply every pending release whose delay has elapsed.

        Returns True when the pressed set changed.
        """
        current = self._now(now)
        due = [key for key, deadline in self._pending.items() if deadline <= current]
        changed = False
        for key in due:
            self._pending.pop(key, None)
            if key in self._pressed:
                self._pressed.discard(key)
                changed = True
        return changed

    def clear(self):
        """Drop all state immediately (safety paths)."""
        changed = bool(self._pressed or self._pending)
        self._pressed.clear()
        self._pending.clear()
        return changed

    def next_deadline(self):
        """Earliest pending release deadline, or None when nothing is pending."""
        return min(self._pending.values()) if self._pending else None

    def _now(self, now):
        return self._clock() if now is None else float(now)


class KeyEventDiagnostics:
    """Opt-in JSON-lines trace of raw key/focus events and published commands.

    Disabled unless ``CONTROL_CENTER_KEYDIAG`` is set to a truthy value.  When
    enabled, appends to ``CONTROL_CENTER_KEYDIAG_PATH`` or
    ``/tmp/control_center_keydiag_<pid>.log``.  Every write is best-effort and
    must never break motion handling.
    """

    def __init__(self, path=None, enabled=None, clock=None):
        if enabled is None:
            enabled = os.environ.get('CONTROL_CENTER_KEYDIAG', '') not in (
                '', '0', 'false', 'False', 'no')
        self._clock = time.monotonic if clock is None else clock
        self._file = None
        self.path = None
        if enabled:
            target = (path or os.environ.get('CONTROL_CENTER_KEYDIAG_PATH')
                      or '/tmp/control_center_keydiag_%d.log' % os.getpid())
            try:
                self._file = open(target, 'a', encoding='utf-8')
            except OSError:
                self._file = None
            else:
                self.path = target

    @property
    def enabled(self):
        return self._file is not None

    def log(self, kind, **fields):
        if self._file is None:
            return
        payload = {'t': round(self._clock(), 6), 'kind': kind}
        payload.update(fields)
        try:
            self._file.write(json.dumps(payload, ensure_ascii=False) + '\n')
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
