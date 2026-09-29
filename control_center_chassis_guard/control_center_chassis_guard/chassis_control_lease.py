"""Chassis control ownership lease for the control center manual driving pages.

The vendor arm controller republishes a zero ``Twist`` on the shared chassis
command topic at ~62.5 Hz unless the Redis key
``shared_params:{domain}:{robot}:is_navigating`` holds the plain string
``true``.  The control center never set that key, so its WASD commands
interleaved with the vendor safety zeros and the chassis stuttered.  This
module implements the control-center side of that ownership contract:

* acquire before the first non-zero manual command (fail closed),
* renew in a background thread; while held the keys carry a short TTL so a
  crashed or powered-off holder cannot suppress the vendor safety zeros for
  longer than ``lease_ttl_sec``,
* release only for the current owner token (atomic Lua check-and-set), so
  one page can never release a lease owned by the other,
* treat any Redis failure as "not allowed to drive" and report it.

Key/value formats follow the vendor ``SharedParams`` contract: DB 0, plain
string ``true``/``false``.  On clean release the vendor key is rewritten as
``false`` without TTL, matching the vendor's own default write style.
"""

import os
import re
import threading
import time
import uuid


DEFAULT_LEASE_TTL_SEC = 0.5
DEFAULT_RENEW_PERIOD_SEC = 0.1
DEFAULT_ACQUIRE_TIMEOUT_SEC = 0.3
DEFAULT_REDIS_HOST = '127.0.0.1'
DEFAULT_REDIS_PORT = 6379
DEFAULT_REDIS_DB = 0
DEFAULT_SOCKET_TIMEOUT_SEC = 0.25

# Atomic take: only succeed when the owner slot is free or already ours, then
# refresh both keys in one round trip.  Used for acquire and renewal.
TAKE_SCRIPT = """\
local owner = redis.call('GET', KEYS[2])
if owner == false or owner == ARGV[1] then
    redis.call('SET', KEYS[2], ARGV[1], 'PX', ARGV[2])
    redis.call('SET', KEYS[1], 'true', 'PX', ARGV[2])
    return 1
end
return 0
"""

# Atomic release: only the current owner may restore the vendor key to the
# vendor-style plain "false" (no TTL) and delete the owner slot.
RELEASE_SCRIPT = """\
local owner = redis.call('GET', KEYS[2])
if owner == ARGV[1] then
    redis.call('SET', KEYS[1], 'false')
    redis.call('DEL', KEYS[2])
    return 1
end
return 0
"""

_SUFFIX_VALUE_RE = re.compile(r'^(\d+)_(\d+)$')
_SUFFIX_TOPIC_RE = re.compile(r'_(\d+)_(\d+)$')


def resolve_chassis_identity(topic_suffix=None, topic_hint=None, environ=None):
    """Resolve ``(domain_id, robot_id)`` strings for the lease keys.

    Precedence: ``ROS_DOMAIN_ID`` / ``ROBOT_ID`` environment variables first,
    then the ``<domain>_<robot>`` topic suffix given directly or extracted
    from a topic name such as ``/topic_gv_front_lidar_0_307``.  Unresolved
    parts come back as empty strings; the lease then fails closed instead of
    guessing a robot id.
    """
    if environ is None:
        environ = os.environ
    domain = str(environ.get('ROS_DOMAIN_ID', '')).strip()
    robot = str(environ.get('ROBOT_ID', '')).strip()
    suffix = None
    if topic_suffix is not None:
        match = _SUFFIX_VALUE_RE.match(str(topic_suffix).strip())
        if match:
            suffix = match.groups()
    elif topic_hint is not None:
        match = _SUFFIX_TOPIC_RE.search(str(topic_hint))
        if match:
            suffix = match.groups()
    if suffix is not None:
        domain = domain or suffix[0]
        robot = robot or suffix[1]
    return domain, robot


def _as_text(value):
    if value is None:
        return None
    if isinstance(value, bytes):
        return value.decode('utf-8', 'replace')
    return str(value)


class ChassisControlLease:
    """Control-center side of the vendor chassis ownership contract."""

    def __init__(self, domain_id=None, robot_id=None, topic_suffix=None,
                 topic_hint=None, redis_client=None, environ=None,
                 redis_host=DEFAULT_REDIS_HOST, redis_port=DEFAULT_REDIS_PORT,
                 redis_db=DEFAULT_REDIS_DB, key_prefix='shared_params',
                 lease_ttl_sec=DEFAULT_LEASE_TTL_SEC,
                 renew_period_sec=DEFAULT_RENEW_PERIOD_SEC,
                 acquire_timeout_sec=DEFAULT_ACQUIRE_TIMEOUT_SEC,
                 owner_token=None, logger=None, on_lease_lost=None):
        lease_ttl_sec = float(lease_ttl_sec)
        renew_period_sec = float(renew_period_sec)
        if lease_ttl_sec <= 0.0:
            raise ValueError('lease_ttl_sec must be positive')
        if renew_period_sec <= 0.0:
            raise ValueError('renew_period_sec must be positive')
        if renew_period_sec * 3.0 > lease_ttl_sec:
            raise ValueError(
                'renew_period_sec must be at most lease_ttl_sec / 3 '
                'so a missed renewal cannot expire the lease between ticks'
            )
        domain, robot = resolve_chassis_identity(
            topic_suffix=topic_suffix, topic_hint=topic_hint, environ=environ,
        )
        # ROS uses domain 0 when ROS_DOMAIN_ID is unset; never guess the robot.
        self.domain_id = str(domain_id) if domain_id is not None else (domain or '0')
        self.robot_id = str(robot_id) if robot_id is not None else robot
        self._vendor_key = '%s:%s:%s:is_navigating' % (
            key_prefix, self.domain_id, self.robot_id)
        self._owner_key = '%s:%s:%s:chassis_owner' % (
            key_prefix, self.domain_id, self.robot_id)
        self._ttl_ms = int(round(lease_ttl_sec * 1000.0))
        self._renew_period = renew_period_sec
        self._acquire_timeout = float(acquire_timeout_sec)
        self._token = owner_token or uuid.uuid4().hex
        self._logger = logger
        self._on_lease_lost = on_lease_lost
        self._state_lock = threading.Lock()
        self._held = False
        self._last_error = None
        self._stop_thread = threading.Event()
        self._thread = None
        self._client = redis_client
        self._owns_client = redis_client is None
        self._client_error = None
        self._script_runners = {}
        if self._client is None:
            try:
                import redis as _redis
            except ImportError as error:  # pragma: no cover - env dependent
                self._client_error = 'python3-redis 未安装：%s' % error
            else:
                self._client = _redis.Redis(
                    host=redis_host, port=redis_port, db=redis_db,
                    socket_timeout=DEFAULT_SOCKET_TIMEOUT_SEC,
                    socket_connect_timeout=DEFAULT_SOCKET_TIMEOUT_SEC,
                )

    @property
    def held(self):
        with self._state_lock:
            return self._held

    @property
    def last_error(self):
        with self._state_lock:
            return self._last_error

    def acquire(self, timeout_sec=None):
        """Take (or keep) the chassis ownership lease.

        Returns True only when the vendor key is ours and read back as
        ``true``.  Any Redis failure, missing robot id or competing owner
        returns False; the caller must then not publish non-zero velocity.
        """
        with self._state_lock:
            if self._held:
                return True
        if self._client_error is not None:
            self._record_error(self._client_error)
            return False
        if self._client is None:
            self._record_error('Redis 客户端不可用。')
            return False
        if not self.robot_id:
            self._record_error(
                '无法确定机器人编号（ROBOT_ID / topic_suffix），拒绝手动底盘控制。')
            return False
        timeout = self._acquire_timeout if timeout_sec is None else float(timeout_sec)
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            reason = None
            try:
                result = int(self._run_script(
                    TAKE_SCRIPT,
                    (self._vendor_key, self._owner_key),
                    (self._token, self._ttl_ms)))
                if result == 1:
                    readback = _as_text(self._client.get(self._vendor_key))
                    if readback != 'true':
                        self._record_error(
                            'is_navigating 写入后读回校验失败：%r' % readback)
                        return False
                    with self._state_lock:
                        self._held = True
                        self._last_error = None
                        self._stop_thread.clear()
                        self._ensure_renewal_thread_locked()
                    return True
                reason = '底盘控制权正被其他控制端持有。'
            except Exception as error:  # fail closed on any Redis error
                reason = 'Redis 操作失败：%s' % error
            if time.monotonic() >= deadline:
                self._record_error(reason)
                return False
            time.sleep(0.02)

    def release(self):
        """Give the chassis back to the vendor safety zeros, if still ours.

        Returns True after a clean token-checked release.  When Redis fails
        or the token no longer matches, the holder simply stops renewing and
        the key TTL restores the vendor zeros within ``lease_ttl_sec``.
        """
        with self._state_lock:
            if not self._held:
                return True
        result = 0
        try:
            result = int(self._run_script(
                RELEASE_SCRIPT,
                (self._vendor_key, self._owner_key),
                (self._token,)))
        except Exception as error:
            self._record_error('Redis 释放失败（将由 TTL 兜底恢复）：%s' % error)
        with self._state_lock:
            self._held = False
            self._stop_thread.set()
        return result == 1

    def shutdown(self):
        """Explicit cleanup: release if held and stop the renewal thread."""
        self.release()
        if self._owns_client and self._client is not None:
            close = getattr(self._client, 'close', None)
            if close is not None:
                try:
                    close()
                except Exception:  # pragma: no cover - best effort
                    pass

    def __enter__(self):
        if not self.acquire():
            raise RuntimeError(self.last_error or '未能获取底盘控制权。')
        return self

    def __exit__(self, exc_type, exc, tb):
        self.release()
        return False

    def _run_script(self, script, keys, args):
        """Run one of the package Lua scripts atomically on the server.

        Uses the redis-py ``register_script`` surface so a shared runner is
        reused (EVALSHA fast path on real Redis).
        """
        runner = self._script_runners.get(script)
        if runner is None:
            runner = self._client.register_script(script)
            self._script_runners[script] = runner
        return runner(keys=list(keys), args=list(args))

    def _ensure_renewal_thread_locked(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._renew_loop, name='chassis-control-lease-renew',
            daemon=True)
        self._thread.start()

    def _renew_loop(self):
        while not self._stop_thread.wait(self._renew_period):
            with self._state_lock:
                if not self._held:
                    return
            try:
                result = int(self._run_script(
                    TAKE_SCRIPT,
                    (self._vendor_key, self._owner_key),
                    (self._token, self._ttl_ms)))
            except Exception as error:
                self._lose_lease('Redis 续租失败：%s' % error)
                return
            if result != 1:
                self._lose_lease('底盘控制权已被其他控制端接管。')
                return

    def _lose_lease(self, reason):
        with self._state_lock:
            if not self._held:
                return
            self._held = False
            self._last_error = reason
            self._stop_thread.set()
        self._log('error', '底盘控制租约丢失：%s' % reason)
        callback = self._on_lease_lost
        if callback is not None:
            try:
                callback(reason)
            except Exception as error:  # pragma: no cover - callback guard
                self._log('error', 'on_lease_lost 回调异常：%s' % error)

    def _record_error(self, message):
        with self._state_lock:
            self._last_error = message
        if message:
            self._log('error', message)

    def _log(self, level, message):
        logger = self._logger
        if logger is None:
            return
        try:
            getattr(logger, level)(message)
        except Exception:  # pragma: no cover - logging must never break motion
            pass
