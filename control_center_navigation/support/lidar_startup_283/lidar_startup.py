"""Bounded lidar-only recovery before the vendor GV ROS service starts.

Installed only in autolife_robot_gv; never used by arm/vision services.
No runtime watchdog, whole-service restart, or motion commands.
"""
import time

LIDARS = {
    'mod_lidar_front': 'get_front_lidar_data',
    'mod_lidar_rear': 'get_rear_lidar_data',
}
SENSOR_MODULES = set(LIDARS) | {'mod_imu_main', 'mod_battery_main'}


def recover_lidars(api, modules, initialize, close, *, retries=2,
                   sleep=time.sleep, now=time.monotonic, log=print):
    """The initial SDK initialization has already completed.

    Use SDK public APIs, retry only unavailable configured lidars. A healthy
    front lidar is not reopened when the rear lidar fails. Leave persistent
    failures visible to navigation; never fabricate scans or downgrade safety.
    """
    pending = [name for name in LIDARS if name in modules]
    def ready(name):
        try:
            data = getattr(api, LIDARS[name])()
            return data is not None and len(data) > 0
        except Exception:
            return False

    for attempt in range(retries + 1):
        deadline = now() + 2.0
        while pending:
            pending = [name for name in pending if not ready(name)]
            if not pending or now() >= deadline:
                break
            sleep(0.1)
        if not pending:
            log('[lidar-startup] configured front/rear readers ready', flush=True)
            return []
        if attempt == retries:
            break
        for name in pending:
            log(f'[lidar-startup] retry {attempt + 1}/{retries}: {name}', flush=True)
            try:
                # close() is scoped to this module, including partially opened
                # readers. Never open a second reader after failed cleanup.
                close(api, [name])
            except Exception as exc:
                log(f'[lidar-startup] cleanup failed; no duplicate reader: {exc}', flush=True)
                continue
            sleep(0.5)
            try:
                initialize(api, [name])
            except Exception as exc:
                log(f'[lidar-startup] initialization failed: {exc}', flush=True)
    log('[lidar-startup] unavailable after bounded retries: ' + ', '.join(pending), flush=True)
    return pending


def install():
    """Wrap this process's SDK initializer, not SDK files or other processes."""
    from autolife_robot_sdk.hardware.hw_api import HWAPI
    original = HWAPI.initialize
    if getattr(original, '_lidar_startup_recovery', False):
        return
    original_close = HWAPI.close

    def initialize(self, modules_to_activate, **kwargs):
        result = original(self, modules_to_activate, **kwargs)
        modules = set(modules_to_activate)
        # The GV configuration on 283 is sensor-only. Do not extend recovery
        # to future configurations containing motors or other hardware.
        if modules & set(LIDARS) and modules <= SENSOR_MODULES:
            recover_lidars(self, modules,
                          lambda api, names: original(api, names, **kwargs),
                          original_close)
        return result

    initialize._lidar_startup_recovery = True
    HWAPI.initialize = initialize
