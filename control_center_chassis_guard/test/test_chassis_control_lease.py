"""Unit tests for ChassisControlLease against FakeRedis (no real Redis)."""

import threading
import time

import pytest

from control_center_chassis_guard.chassis_control_lease import (
    ChassisControlLease,
    TAKE_SCRIPT,
    RELEASE_SCRIPT,
)

from fake_redis import FakeRedis


FAST = dict(lease_ttl_sec=0.3, renew_period_sec=0.05, acquire_timeout_sec=0.05)
VENDOR = 'shared_params:0:306:is_navigating'
OWNER = 'shared_params:0:306:chassis_owner'


def make_lease(fake, **kwargs):
    kwargs.setdefault('robot_id', '306')
    kwargs.setdefault('domain_id', '0')
    kwargs.setdefault('redis_client', fake)
    kwargs.update(FAST)
    return ChassisControlLease(**kwargs)


class TestAcquire:
    def test_acquire_sets_keys_with_ttl_and_readback(self):
        fake = FakeRedis()
        lease = make_lease(fake)
        assert lease.acquire() is True
        assert fake.get(VENDOR) == b'true'
        assert fake.get(OWNER) == lease._token.encode()
        assert 0 < fake.ttl_ms(VENDOR) <= 300
        assert lease.held is True
        assert lease.last_error is None

    def test_acquire_is_idempotent_while_held(self):
        fake = FakeRedis()
        lease = make_lease(fake)
        assert lease.acquire() is True
        assert lease.acquire() is True

    def test_acquire_fails_closed_without_robot_id(self):
        fake = FakeRedis()
        lease = ChassisControlLease(
            redis_client=fake, domain_id='0', robot_id='', environ={},
            **FAST)
        assert lease.domain_id == '0'
        assert lease.acquire() is False
        assert '机器人编号' in lease.last_error

    def test_acquire_fails_closed_on_persistent_redis_error(self):
        fake = FakeRedis()
        fake.always_fail = ConnectionError('boom')
        lease = make_lease(fake)
        assert lease.acquire() is False
        assert 'Redis 操作失败' in lease.last_error

    def test_acquire_fails_closed_when_readback_disagrees(self):
        fake = FakeRedis()
        lease = make_lease(fake)
        original_get = fake.get

        def lying_get(key):
            if key == VENDOR:
                return b'false'
            return original_get(key)

        fake.get = lying_get
        assert lease.acquire() is False
        assert '读回校验失败' in lease.last_error

    def test_acquire_reports_missing_redis_module(self, monkeypatch):
        import sys
        monkeypatch.setitem(sys.modules, 'redis', None)
        lease = ChassisControlLease(robot_id='306', domain_id='0', **FAST)
        assert lease.acquire() is False
        assert 'python3-redis 未安装' in lease.last_error


class TestMutualExclusion:
    def test_second_lease_cannot_take_while_first_holds(self):
        fake = FakeRedis()
        first = make_lease(fake)
        second = ChassisControlLease(
            redis_client=fake, domain_id='0', robot_id='306',
            owner_token='other-token', **FAST)
        assert first.acquire() is True
        assert second.acquire(timeout_sec=0.05) is False
        assert '其他控制端' in second.last_error
        assert fake.get(VENDOR) == b'true'
        first.shutdown()
        assert second.acquire(timeout_sec=0.2) is True
        second.shutdown()

    def test_release_restores_vendor_style_false_without_ttl(self):
        fake = FakeRedis()
        lease = make_lease(fake)
        assert lease.acquire() is True
        assert lease.release() is True
        assert fake.get(VENDOR) == b'false'
        assert fake.ttl_ms(VENDOR) == -1
        assert fake.get(OWNER) is None
        assert lease.held is False

    def test_foreign_token_cannot_release(self):
        fake = FakeRedis()
        lease = make_lease(fake)
        assert lease.acquire() is True
        # Simulate another control center taking over the owner slot.
        fake.set(OWNER, 'someone-else')
        assert lease.release() is False
        assert fake.get(VENDOR) == b'true'
        assert lease.held is False

    def test_release_is_noop_when_not_held(self):
        fake = FakeRedis()
        lease = make_lease(fake)
        assert lease.release() is True
        assert fake.get(VENDOR) is None


class TestRenewalAndRecovery:
    def test_renewal_keeps_keys_alive_beyond_ttl(self):
        fake = FakeRedis()
        lease = make_lease(fake)
        assert lease.acquire() is True
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            if fake.get(VENDOR) != b'true':
                pytest.fail('vendor key expired while lease was renewed')
            time.sleep(0.05)
        assert lease.held is True
        lease.shutdown()

    def test_crashed_holder_expires_via_ttl(self):
        # Simulate a crashed process: renewal stops without any release.
        fake = FakeRedis()
        lease = make_lease(fake)
        assert lease.acquire() is True
        lease._stop_thread.set()  # kill the renewal thread only
        time.sleep(0.45)
        assert fake.get(VENDOR) is None  # TTL recovered the vendor zeros
        assert fake.get(OWNER) is None

    def test_renewal_failure_invokes_on_lease_lost(self):
        fake = FakeRedis()
        lost = threading.Event()
        reasons = []
        lease = make_lease(
            fake, on_lease_lost=lambda reason: (reasons.append(reason),
                                                lost.set()))
        assert lease.acquire() is True
        fake.fail_next = ConnectionError('boom')
        assert lost.wait(timeout=2.0), 'on_lease_lost was not invoked'
        assert lease.held is False
        assert 'Redis 续租失败' in lease.last_error

    def test_takeover_invokes_on_lease_lost(self):
        fake = FakeRedis()
        lost = threading.Event()
        lease = make_lease(fake, on_lease_lost=lambda reason=None: lost.set())
        assert lease.acquire() is True
        fake.set(OWNER, 'someone-else')
        assert lost.wait(timeout=2.0), 'on_lease_lost was not invoked'
        assert lease.held is False
        assert '其他控制端' in lease.last_error


class TestConfigValidation:
    def test_renew_period_must_leave_ttl_headroom(self):
        fake = FakeRedis()
        with pytest.raises(ValueError):
            ChassisControlLease(
                redis_client=fake, domain_id='0', robot_id='306',
                lease_ttl_sec=0.1, renew_period_sec=0.05)

    def test_scripts_match_package_constants(self):
        # The atomic scripts are part of the safety contract; keep the test
        # fake and the package implementation locked to the same bodies.
        fake = FakeRedis()
        runner = fake.register_script(TAKE_SCRIPT)
        assert callable(runner)
        assert callable(fake.register_script(RELEASE_SCRIPT))
