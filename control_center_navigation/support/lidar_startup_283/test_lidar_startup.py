import unittest
from lidar_startup import recover_lidars

class Clock:
    value = 0.0
    def now(self): return self.value
    def sleep(self, seconds): self.value += seconds

class API:
    def __init__(self, failed=(), permanent=False):
        self.failed = set(failed)
        self.permanent = permanent
        self.calls = []
    def data(self, name):
        if name in self.failed: raise ValueError('module not found')
        return {'ranges': [1.0]}
    def get_front_lidar_data(self): return self.data('mod_lidar_front')
    def get_rear_lidar_data(self): return self.data('mod_lidar_rear')
    def initialize(self, names):
        self.calls.append(('initialize', names[:]))
        if not self.permanent: self.failed.difference_update(names)
    def close(self, names): self.calls.append(('close', names[:]))

class Tests(unittest.TestCase):
    def run_recovery(self, api, modules=('mod_lidar_front','mod_lidar_rear')):
        clock = Clock()
        return recover_lidars(api, modules, API.initialize, API.close,
                              sleep=clock.sleep, now=clock.now,
                              log=lambda *a, **k: None)
    def test_healthy_not_reopened(self):
        a=API(); self.assertEqual(self.run_recovery(a), []); self.assertEqual(a.calls, [])
    def test_only_failed_rear_retried(self):
        a=API(['mod_lidar_rear']); self.assertEqual(self.run_recovery(a), [])
        self.assertEqual(a.calls,[('close',['mod_lidar_rear']),('initialize',['mod_lidar_rear'])])
    def test_permanent_bounded(self):
        a=API(['mod_lidar_rear'],True)
        self.assertEqual(self.run_recovery(a), ['mod_lidar_rear'])
        self.assertEqual(len(a.calls),4)
    def test_unconfigured_not_opened(self):
        a=API(['mod_lidar_rear']); self.assertEqual(self.run_recovery(a, ['mod_lidar_front']), [])
        self.assertEqual(a.calls,[])
    def test_cleanup_failure_never_opens_duplicate(self):
        a=API(['mod_lidar_rear']); clock=Clock()
        def fail_close(*args): raise RuntimeError('busy')
        missing=recover_lidars(a, ['mod_lidar_rear'], API.initialize, fail_close,
            sleep=clock.sleep, now=clock.now, log=lambda *a, **k: None)
        self.assertEqual(missing,['mod_lidar_rear']); self.assertEqual(a.calls,[])

if __name__ == '__main__': unittest.main()
