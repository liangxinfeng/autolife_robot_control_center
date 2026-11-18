"""Integration tests against the local Redis server using ISOLATED keys only.

These tests never touch the production ``shared_params:{domain}:{robot}:*``
keys: they run with ``key_prefix='zcode_test_chassis_guard'`` and assert at
the end that the production vendor key is unchanged.  The whole module
self-skips when the redis module or the local server is unavailable.
"""

import os

import pytest

redis = pytest.importorskip('redis')

client = redis.Redis(
    host='127.0.0.1', port=6379, db=0,
    socket_timeout=0.25, socket_connect_timeout=0.25)
try:
    client.ping()
except Exception:  # pragma: no cover - environment dependent
    pytest.skip('local Redis server unreachable', allow_module_level=True)

from control_center_chassis_guard.chassis_control_lease import (  # noqa: E402
    ChassisControlLease,
)

PREFIX = 'zcode_test_chassis_guard'
# The production key differs per robot (ROBOT_ID env; default keeps the
# original 306 deployment behaviour when the env var is absent).
ROBOT = os.environ.get('ROBOT_ID') or '306'
PROD_VENDOR = 'shared_params:0:%s:is_navigating' % ROBOT
VENDOR = PREFIX + ':0:%s:is_navigating' % ROBOT
OWNER = PREFIX + ':0:%s:chassis_owner' % ROBOT
FAST = dict(lease_ttl_sec=0.3, renew_period_sec=0.05, acquire_timeout_sec=0.3)


@pytest.fixture(autouse=True)
def _cleanup_keys():
    yield
    client.delete(VENDOR, OWNER)


def _lease(token=None):
    return ChassisControlLease(
        redis_client=client, domain_id='0', robot_id=ROBOT,
        key_prefix=PREFIX, owner_token=token, **FAST)


def test_lifecycle_on_real_redis():
    lease = _lease()
    assert lease.acquire() is True
    assert client.get(VENDOR) == b'true'
    assert 0 < client.pttl(VENDOR) <= 300
    assert lease.release() is True
    assert client.get(VENDOR) == b'false'
    assert client.exists(OWNER) == 0


def test_mutual_exclusion_on_real_redis():
    first = _lease('token-a')
    second = _lease('token-b')
    assert first.acquire() is True
    assert second.acquire(timeout_sec=0.05) is False
    first.shutdown()
    assert second.acquire(timeout_sec=0.5) is True
    second.shutdown()


def test_production_vendor_key_untouched():
    before = client.get(PROD_VENDOR)
    lease = _lease()
    assert lease.acquire() is True
    lease.shutdown()
    assert client.get(PROD_VENDOR) == before
