"""Minimal in-memory stand-in for the redis client used by ChassisControlLease.

Covers exactly the command surface the lease touches: GET, SET (with PX),
DEL and ``register_script`` for the two package Lua scripts.  The script
dispatch mirrors the Lua bodies so tests exercise the same atomic-take /
token-checked-release logic that runs on real Redis.
"""

import time


def _b(value):
    return value.encode() if isinstance(value, str) else value


class FakeRedis:
    def __init__(self, clock=time.monotonic):
        self.data = {}
        self.expiry = {}
        self.clock = clock
        self.fail_next = None
        self.always_fail = None

    # -- helpers ---------------------------------------------------------
    def _check(self):
        error = self.fail_next
        if error is not None:
            self.fail_next = None
        else:
            error = self.always_fail
        if error is not None:
            raise error

    def _sweep(self, key):
        deadline = self.expiry.get(key)
        if deadline is not None and self.clock() >= deadline:
            self.data.pop(key, None)
            self.expiry.pop(key, None)

    # -- redis surface ---------------------------------------------------
    def get(self, key):
        self._check()
        self._sweep(key)
        return self.data.get(key)

    def set(self, key, value, px=None, ex=None):
        self._check()
        self.data[key] = _b(value)
        if px is not None:
            self.expiry[key] = self.clock() + px / 1000.0
        elif ex is not None:
            self.expiry[key] = self.clock() + float(ex)
        else:
            self.expiry.pop(key, None)
        return True

    def delete(self, *keys):
        self._check()
        removed = 0
        for key in keys:
            self._sweep(key)
            if self.data.pop(key, None) is not None:
                removed += 1
            self.expiry.pop(key, None)
        return removed

    def ttl_ms(self, key):
        """Remaining TTL in milliseconds, or -1 when the key has no TTL."""
        self._sweep(key)
        if key not in self.data:
            return -2
        deadline = self.expiry.get(key)
        return -1 if deadline is None else int((deadline - self.clock()) * 1000)

    def register_script(self, script):
        from control_center_chassis_guard.chassis_control_lease import (
            RELEASE_SCRIPT,
            TAKE_SCRIPT,
        )

        def run(keys=None, args=None, client=None):
            self._check()
            keys = list(keys or [])
            args = list(args or [])
            if script == TAKE_SCRIPT:
                vendor_key, owner_key = keys
                token, ttl_ms = args[0], int(args[1])
                owner = self.get(owner_key)
                if owner is None or owner == _b(token):
                    self.set(owner_key, token, px=ttl_ms)
                    self.set(vendor_key, 'true', px=ttl_ms)
                    return 1
                return 0
            if script == RELEASE_SCRIPT:
                vendor_key, owner_key = keys
                token = args[0]
                owner = self.get(owner_key)
                if owner == _b(token):
                    self.set(vendor_key, 'false')
                    self.delete(owner_key)
                    return 1
                return 0
            raise AssertionError('unexpected script passed to FakeRedis')

        return run

    def close(self):
        return None
