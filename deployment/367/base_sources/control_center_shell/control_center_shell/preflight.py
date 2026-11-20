"""Common read-only checks and explicit supervisor-level recovery."""

import json


import math


import os


from collections import deque


from pathlib import Path


import re


import signal


import subprocess


import tempfile


import time


from contextlib import contextmanager


STATE_DIR = Path.home() / '.local/state/autolife-control-center/processes'


GV_CONTROL_SERVICE = 'gv-control-service.service'


ARM_CONTROL_SERVICE = 'arm-control-service.service'


GV_SLAM_SERVICE = 'gv-slam-service.service'


KEYS = ('navigation', 'slam', 'joint')


LIDAR_RECOVERY_TIMEOUT = 20.0


@contextmanager
def _probe_node(name):
    """Keep worker probes off the GUI's context and global executor."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    context = Context()
    node = None
    executor = None
    try:
        rclpy.init(context=context)
        node = rclpy.create_node(name, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        yield node, executor
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.try_shutdown()


LAUNCH = {
    'navigation': 'control_center_navigation bringup_all.launch.py',
    'slam': 'control_center_slam multi_lidar_slam_optimized.launch.py',
    'joint': 'control_center_joint_control whole_body_joint_control.launch.py',
}


SINGLE_NODES = {
    'chassis_control': ('/control_center_navigation/cmd_vel_arbiter_node.py',),
    'joint_control': ('/control_center_joint_control/joint_bridge',
                      '/control_center_joint_control/lib/control_center_joint_control/joint_bridge'),
    'dual_lidar': ('/control_center_slam/mapping_scan_adapter.py',),
}


MODULE_HEALTH_MARKERS = {
    'navigation': (
        ('地图服务', '/nav2_map_server/map_server'),
        ('定位服务', 'amcl_startup_compat'),
        ('路径规划', '/nav2_planner/planner_server'),
        ('运动控制', '/nav2_controller/controller_server'),
        ('底盘仲裁', 'cmd_vel_arbiter_node.py'),
    ),
    'slam': (
        ('雷达适配', '/control_center_slam/mapping_scan_adapter.py'),
        ('建图核心', 'cartographer_multi_origin_node'),
        ('地图发布', 'cartographer_occupancy_grid_node'),
    ),
}


def _processes():
    result = {}
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            process = _proc(int(entry.name))
            if process and process['command']:
                result[process['pid']] = process
    return result


def _descends(pid, ancestor, processes):
    seen = set()
    while pid in processes and pid not in seen:
        if pid == ancestor:
            return True
        seen.add(pid)
        pid = processes[pid]['ppid']
    return False


def _group(key, record, processes):
    if (not isinstance(record, dict)
            or type(record.get('pid')) is not int
            or type(record.get('start')) is not int):
        return None
    leader = processes.get(record.get('pid'))
    if not leader or leader['start'] != record.get('start') or leader['pgid'] != leader['pid']:
        return None
    if LAUNCH[key] not in leader['command'] or leader['uid'] != os.getuid():
        return None
    members = [p for p in processes.values() if p['pgid'] == leader['pid']]
    if not members or any(p['uid'] != os.getuid() or
                          not _descends(p['pid'], leader['pid'], processes)
                          for p in members):
        return None
    return members


def record_process(key, pid):
    process = _proc(pid)
    if key not in KEYS or not process or process['pgid'] != pid:
        return
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    target = STATE_DIR / (key + '.json')
    fd, temporary_name = tempfile.mkstemp(
        prefix=f'.{key}.', suffix='.tmp', dir=STATE_DIR
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(json.dumps({'pid': pid, 'start': process['start']}))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
        directory_fd = os.open(STATE_DIR, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except OSError:
        temporary.unlink(missing_ok=True)
        raise


def _quarantine_process_record(key, target):
    """Preserve an unreadable record without trusting it for process cleanup."""
    quarantine = target.with_name(
        f'{key}.corrupt.{time.time_ns()}.{os.getpid()}.json'
    )
    try:
        os.replace(target, quarantine)
    except FileNotFoundError:
        return None
    except OSError as error:
        return f'无法隔离旧记录 {target.name}：{error}'
    return quarantine.name


def forget_process(key):
    if key in KEYS:
        (STATE_DIR / (key + '.json')).unlink(missing_ok=True)


def stop_managed_group(key, group_id, emit=print):
    """Stop one control-center-owned group after revalidating its identity."""
    if key not in KEYS or not group_id:
        return False
    leader = _proc(int(group_id))
    if leader is None:
        forget_process(key)
        return True
    record = {'pid': int(group_id), 'start': leader['start']}
    if _group(key, record, _processes()) is None:
        emit(f'{key}: repair cleanup refused; group ownership changed')
        return False
    if not _terminate_group(key, record, emit):
        return False
    forget_process(key)
    return True


def _terminate_group(key, record, emit):
    pid = record['pid']
    for signum, timeout in ((signal.SIGINT, 2.0), (signal.SIGTERM, 1.0),
                            (signal.SIGKILL, 0.5)):
        processes = _processes()
        members = _group(key, record, processes)
        if members is None:
            if not any(p['pgid'] == pid for p in processes.values()):
                emit(f'{key}: cleanup verified pid={pid}')
                return True
            return False
        emit(f'{key}: cleanup signal={signum.name} pid={pid} members={len(members)}')
        try:
            os.killpg(pid, signum)
        except ProcessLookupError:
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not any(p['pgid'] == pid for p in _processes().values()):
                emit(f'{key}: cleanup verified pid={pid}')
                return True
            time.sleep(0.05)
    return not any(p['pgid'] == pid for p in _processes().values())


def _conflicts(processes, suffix='0_367'):
    own = [p for p in processes.values() if p['uid'] == os.getuid()]
    result = {}
    for key, marker in LAUNCH.items():
        matched = [p for p in own if marker in p['command']]
        if matched:
            result[key] = matched
    for key, markers in SINGLE_NODES.items():
        matched = [p for p in own if (
                       any(marker in p['command'] for marker in markers)
                       or (key == 'dual_lidar'
                           and '/dual_laser_merger/dual_laser_merger_node' in p['command']
                           and f'__node:=dual_laser_merger_{suffix}' in p['command']))
                   and not (key == 'dual_lidar' and
                            'gv-slam-service.service' in p.get('cgroup', ''))]
        if matched:
            result[key] = matched
    legacy = [p for p in own if 'whole_body_joint_control' in p['command']
              and ('joint_bridge' in p['command'] or 'joint_control_panel' in p['command'])
              and 'control_center_joint_control' not in p['command']]
    if legacy:
        result['legacy_joint'] = legacy
    return result


def _service_state(name):
    try:
        result = subprocess.run(
            ['systemctl', '--user', 'is-active', name],
            capture_output=True, text=True, timeout=2.0, check=False)
        return result.stdout.strip() or 'unknown'
    except (OSError, subprocess.TimeoutExpired):
        return 'unknown'


def _gv_control_state():
    return _service_state(GV_CONTROL_SERVICE)


def _service_recovery_guard(suffix, processes=None):
    """Return an error when restarting the shared GV service is unsafe."""
    if _service_state(GV_SLAM_SERVICE) == 'active':
        return '厂商导航服务正在运行'
    if processes is None:
        processes = _processes()
    control_markers = ('openarmx_teleop', 'vr_navigation',
                       'record_lerobot', 'groot_policy_bridge')
    external = [p for p in processes.values()
                if p['uid'] == os.getuid()
                and any(marker in p['command'] for marker in control_markers)]
    if external:
        return '检测到其他控制任务 PID ' + ', '.join(
            str(p['pid']) for p in external[:4])
    try:
        import redis
        match = str(suffix).split('_', 1)
        if len(match) != 2 or not all(part.isdigit() for part in match):
            return '无法从话题后缀确认底盘所有权键'
        client = redis.Redis(host='127.0.0.1', port=6379, db=0,
                             socket_timeout=0.25,
                             socket_connect_timeout=0.25)
        prefix = f'shared_params:{match[0]}:{match[1]}'
        navigating = client.get(prefix + ':is_navigating')
        owner = client.get(prefix + ':chassis_owner')
        client.close()
        navigating = navigating.decode(errors='replace') if navigating else ''
        owner = owner.decode(errors='replace') if owner else ''
        if owner or navigating.lower() == 'true':
            return '底盘控制权正在被使用'
    except Exception as error:
        return f'无法确认底盘空闲：{error}'
    return ''


def _recover_gv_control(suffix, action, emit, timeout=12.0):
    """Start/restart the vendor GV service once, only while controls are idle."""
    reason = _service_recovery_guard(suffix)
    if reason:
        emit(f'gv-control-service: recovery blocked: {reason}')
        return False, reason
    emit(f'gv-control-service: AUTO_RECOVERY action={action}')
    try:
        subprocess.run(
            ['systemctl', '--user', 'reset-failed', GV_CONTROL_SERVICE],
            capture_output=True, text=True, timeout=3.0, check=False)
        result = subprocess.run(
            ['systemctl', '--user', action, GV_CONTROL_SERVICE],
            capture_output=True, text=True, timeout=8.0, check=False)
    except (OSError, subprocess.TimeoutExpired) as error:
        return False, f'执行 {action} 失败：{error}'
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        return False, f'执行 {action} 失败：{detail or result.returncode}'
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if _gv_control_state() == 'active':
            emit('gv-control-service: service active after recovery')
            return True, ''
        time.sleep(0.1)
    return False, f'{action} 后 {timeout:.0f} 秒内服务未进入 active'


def _scan_pair(latest, now):
    """Match the relocalizer's configured acquisition-time gate."""
    for front_stamp, front_valid in reversed(latest['front']):
        if not -0.05 <= now-front_stamp <= 0.60:
            continue
        for rear_stamp, rear_valid in reversed(latest['rear']):
            difference = abs(front_stamp-rear_stamp)
            if (-0.05 <= now-rear_stamp <= 0.60 and difference <= 0.10
                    and front_valid+rear_valid >= 60):
                return difference, front_valid+rear_valid
    return None


def _check_lidars(suffix, timeout, emit):
    latest = {'front': deque(maxlen=8), 'rear': deque(maxlen=8)}
    try:
        with _probe_node('control_center_preflight_lidar') as (node, executor):
            return _read_lidars(node, executor, suffix, timeout, emit, latest)
    except Exception as error:
        return f'双雷达查询失败：{error}'


def _lidar_frame(side, suffix):
    # 345 has a front 3D projector with its own level scan frame. Keep the
    # S2 contract strict; accepting arbitrary frame IDs can hide a bad TF.
    if str(suffix).split('_')[-1] == '367' and side == 'front':
        return 's3_front_scan_frame'
    return side + '_laser_frame'


def _read_lidars(node, executor, suffix, timeout, emit, latest):
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    observed = {}
    def receive(side, message):
        expected = _lidar_frame(side, suffix)
        observed[side] = message.header.frame_id
        if message.header.frame_id != expected:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        valid = sum(math.isfinite(value) and
                    max(0.2, message.range_min) < value < min(15.0, message.range_max)
                    for value in message.ranges)
        if stamp > 0 and valid > 0:
            latest[side].append((stamp, valid))
    for side in ('front', 'rear'):
        node.create_subscription(LaserScan, f'/topic_gv_{side}_lidar_{suffix}',
                                 lambda msg, s=side: receive(s, msg), qos)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=min(0.1, max(0, deadline-time.monotonic())))
        now = node.get_clock().now().nanoseconds * 1e-9
        pair = _scan_pair(latest, now)
        if pair is not None:
            difference, points = pair
            emit('dual_lidar: PASS front/rear valid scans pair_age=' +
                 f'{difference:.3f}s points={points} frames={observed}')
            return None
    mismatch = [f'{side} 实际={frame!r}，预期={_lidar_frame(side, suffix)!r}'
                for side, frame in observed.items()
                if frame != _lidar_frame(side, suffix)]
    if mismatch:
        return '雷达坐标系不匹配：' + '；'.join(mismatch)
    missing = [side for side in ('front', 'rear') if not latest[side]]
    if missing:
        detail = '激光雷达未输出有效扫描：' + '、'.join(missing)
        if str(suffix).split('_')[-1] == '367' and 'front' in missing:
            detail += '；请检查前置3D点云和 robot367-s3-lidar.service 转换服务'
        return detail
    return '双雷达扫描未在 0.10 秒内同步'


def _joint_feedback_problem(data):
    from control_center_joint_control.joint_definitions import (
        FEEDBACK_KEYS, GRIPPER_FEEDBACK_KEYS, group_joint_names,
    )
    if not isinstance(data, dict):
        return '关节反馈不是对象'
    groups = [(FEEDBACK_KEYS[group], group_joint_names(group))
              for group in ('leg_waist', 'left_arm', 'right_arm', 'neck')]
    groups.extend((key, [name]) for name, key in GRIPPER_FEEDBACK_KEYS.items())
    lost = []
    for key, names in groups:
        try:
            state = data[key]
            positions = state['position']
            if len(positions) != len(names) or not all(
                    math.isfinite(float(value)) for value in positions):
                return f'{key} 关节位置不完整或无效'
            flags = state.get('communication_lost', [])
            lost.extend(name for index, name in enumerate(names)
                        if index < len(flags) and flags[index])
        except (KeyError, TypeError, ValueError, OverflowError):
            return f'{key} 关节反馈格式无效'
    if lost:
        return '关节通信已断开：' + '、'.join(lost)
    return None


def _check_joint_feedback(suffix, timeout=3.0, emit=print):
    """Require live, complete hardware feedback; service active is insufficient."""
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    topic = f'/topic_arm_whole_body_and_gripper_current_joints_status_{suffix}'
    samples = 0
    last_problem = ''

    def receive(message):
        nonlocal samples, last_problem
        try:
            last_problem = _joint_feedback_problem(json.loads(message.data)) or ''
        except (TypeError, ValueError):
            last_problem = '关节反馈不是有效 JSON'
        samples = 0 if last_problem else samples + 1

    try:
        with _probe_node('control_center_preflight_joints') as (node, executor):
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            node.create_subscription(String, topic, receive, qos)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=min(0.1, max(0, deadline-time.monotonic())))
                if samples >= 2:
                    emit('joint_feedback: PASS live 23-joint feedback, communication normal')
                    return None
            if last_problem:
                return last_problem
            service_state = _service_state(ARM_CONTROL_SERVICE)
            return (f'未收到完整全身关节反馈（{ARM_CONTROL_SERVICE}={service_state}）；'
                    '关节解锁不可用，请检查机械臂服务与 DDS 日志')
    except Exception as error:
        return f'关节反馈查询失败：{error}'


NEVER_STOP_UNITS = frozenset({
    GV_CONTROL_SERVICE, 'arm-control-service.service',
    'autolife_robot_kiosk.service', 'autolife-logo-display.service',
    'autolife-relay.service', 'autolife-admin-build.service',
    'autolife-speaker-mixer-guard.service', 'dashboard-backend.service',
    'void-cog-h5.service',
    'robot367-s3-lidar.service', 'vision-service.service', 'robot-action-service.service',
})


def _chassis_keys(suffix):
    match = str(suffix).split('_', 1)
    if len(match) != 2 or not all(part.isdigit() for part in match):
        return None
    prefix = f'shared_params:{match[0]}:{match[1]}'
    return prefix + ':is_navigating', prefix + ':chassis_owner'


def _redis_client():
    import redis
    return redis.Redis(host='127.0.0.1', port=6379, db=0,
                       socket_timeout=0.3, socket_connect_timeout=0.3)


def _chassis_state(suffix):
    """Read the vendor chassis ownership keys (bounded, read-only)."""
    keys = _chassis_keys(suffix)
    if keys is None:
        raise ValueError(f'话题后缀 {suffix} 无法解析为底盘所有权键')
    client = _redis_client()
    try:
        navigating = client.get(keys[0])
        owner = client.get(keys[1])
    finally:
        client.close()

    def text(value):
        return value.decode(errors='replace') if value else ''
    return text(navigating), text(owner), keys


def check_module_health(module, group_id, timeout=12.0, emit=print, active_groups=None):
    """Verify a restarted navigation or mapping group without moving the robot."""
    if module not in MODULE_HEALTH_MARKERS:
        return f'不支持的模块：{module}'
    try:
        with _probe_node(f'control_center_{module}_repair_check') as (node, executor):
            return _module_health_with_node(node, executor, module, group_id, timeout, emit, active_groups)
    except Exception as error:
        return f'{"导航" if module == "navigation" else "建图"}状态验证失败：{error}'


def _module_health_with_node(node, executor, module, group_id, timeout, emit, active_groups=None):
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    received_map = False
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def receive_map(_message):
        nonlocal received_map
        received_map = True

    node.create_subscription(OccupancyGrid, '/map', receive_map, qos)
    deadline = time.monotonic() + timeout
    last_missing = []
    while time.monotonic() < deadline:
        all_processes = _processes()
        processes = [
            process for process in all_processes.values()
            if _observer.managed_process(process, {int(group_id)}, all_processes)
        ]
        last_missing = [
            label for label, marker in MODULE_HEALTH_MARKERS[module]
            if not any(marker in process['command'] for process in processes)
        ]
        executor.spin_once(
            timeout_sec=min(0.1, max(0, deadline-time.monotonic()))
        )
        if not last_missing and received_map and module_snapshot(
                module, '0_367', dict(active_groups or {}, **{module: int(group_id)})).ok:
            emit(f'{module}: PASS critical processes and /map')
            return None
    if last_missing:
        return '关键进程未运行：' + '、'.join(last_missing)
    return f'{"导航" if module == "navigation" else "建图"}启动后未输出 /map'


# Hardware and process-identity helpers above are shared by all entrypoints.
from . import resource_observer as _observer
from . import resource_repair as _repair

def _proc(pid):
    return _observer.read_process(pid)

def module_process_conflicts(module):
    keys = {'navigation': ('navigation', 'chassis_control'),
            'slam': ('slam', 'dual_lidar'),
            'joint': ('joint', 'joint_control', 'legacy_joint')}.get(module, ())
    conflicts = _conflicts(_processes())
    return list({process['pid']: process for key in keys
                 for process in conflicts.get(key, [])}.values())

def set_local_lease_tokens(tokens):
    _observer.LOCAL_TOKENS = frozenset(str(x) for x in tokens if x)


def close_observer():
    _observer.GRAPH.close()


def paused_services():
    return _repair.load()


def restore_paused(emit=print):
    try:
        return _repair.restore(emit)
    except Exception as error:
        return 'WARN', f'恢复未完成：{error}'

def repair_plan(suffix='0_367', active_groups=None):
    snapshot = resource_snapshot(suffix, active_groups)
    units, groups, blocked = _repair.plan(snapshot)
    linked_units = { _repair.unit_key(member): member
                     for unit, _ in units for member in _repair.related_units(unit) }
    _repair.PREVIEWED = (time.monotonic(),
        {(u.manager, u.uid, u.name, u.cgroup) for u in linked_units.values()},
        {(p['pid'], p['start']) for p in groups})
    descriptions = [f"暂停 {unit.manager} 服务：{unit.name}" for unit, _ in units]
    direct = {_repair.unit_key(unit) for unit, _ in units}
    descriptions.extend(f"关联停止并限制重启：{unit.manager}/{unit.name}"
                        for key, unit in linked_units.items() if key not in direct)
    for entry in _repair.load():
        name = entry['unit']['name']
        if entry.get('state') == 'paused' and not any(u.name == name for u, _ in units):
            descriptions.append(f"维持 {name} 的临时接管条件，直到手动恢复")
    descriptions.extend(f"结束独立进程组：{p['pid']}" for p in groups)
    if not descriptions:
        descriptions.append("未发现可确认归属的外部服务；只检查并处理可证明的旧记录或依赖故障")
    if blocked:
        descriptions.append("仍待确认：" + '；'.join(blocked))
    return ("本次资源修复计划：\n" + '\n'.join(descriptions)
            + "\n\n仅处理以上已确认单元及其子进程；H5 主站和硬件保护服务不在停止范围。"
            "暂停服务会使用临时启动条件阻止重新拉起，直到手动恢复；"
            "不会自动恢复旧导航目标。可取消，或修复后手动恢复被暂停服务。")


def resource_snapshot(suffix='0_367', active_groups=None):
    snapshot = _observer.capture(suffix, active_groups)
    try:
        snapshot.issues.extend(_repair.paused_issues())
    except Exception as error:
        snapshot.issues.append(_observer.Issue('暂停记录', 'UNKNOWN', str(error)))
    return snapshot

def module_snapshot(module, suffix='0_367', active_groups=None):
    snapshot = resource_snapshot(suffix, active_groups)
    if module == 'joint':
        relevant = lambda i: (i.resource in ('ROS', 'joint', 'joint_control', 'legacy_joint')
                              or '/topic_arm_' in i.resource)
    else:
        relevant = lambda i: (i.resource in ('ROS', '/map', '底盘租约', '底盘导航',
                                             'navigation', 'slam', 'dual_lidar', 'chassis_control')
                              or '/topic_gv_target_cmd_vel_' in i.resource
                              or i.state == 'REOCCUPIED')
    snapshot.issues = [i for i in snapshot.issues if relevant(i)]
    return snapshot


def _map_publishers(timeout=1.2):
    endpoints = _observer.GRAPH.read(['/map'], timeout=max(.5, timeout))
    if any(not e.gid or '_UNKNOWN_' in e.name or '_NODE_NAME_UNKNOWN_' in e.name
           or '_NODE_NAMESPACE_UNKNOWN_' in e.name for e in endpoints):
        raise RuntimeError('地图端点仍在发现中，不能判为空闲')
    return [e.name for e in endpoints]


def _external_map_publishers(active_groups=None):
    endpoints = _observer.GRAPH.read(['/map'])
    snapshot = _observer.inspect_endpoints(endpoints, _processes(), active_groups or {}, '0_367')
    issues = [i for i in snapshot.issues if i.resource == '/map']
    if any(i.state in ('DISCOVERING', 'UNKNOWN') for i in issues):
        raise RuntimeError('；'.join(i.detail for i in issues))
    return [e.name for e in endpoints] if issues else []


def _record_issues(active_groups):
    issues = []
    processes = _processes()
    for key in KEYS:
        target = STATE_DIR / (key + '.json')
        if not target.exists():
            continue
        try:
            record = json.loads(target.read_text())
            if (not isinstance(record, dict) or type(record.get('pid')) is not int
                    or type(record.get('start')) is not int):
                raise ValueError('进程记录字段无效')
            members = _group(key, record, processes)
            if members and active_groups.get(key) == record['pid']:
                continue
            issues.append(f'{key} 旧记录或进程组未由本中心管理')
        except (OSError, ValueError) as error:
            issues.append(f'{key} 进程记录不可验证：{error}')
    return issues


def _diagnose(suffix, active_groups, emit, lidar_timeout=5.0):
    snapshot = resource_snapshot(suffix, active_groups)
    if not snapshot.ok:
        emit('resource_scan: ' + snapshot.detail)
    problems = []
    problems.extend(_record_issues(active_groups or {}))
    if _gv_control_state() != 'active':
        problems.append('底盘/雷达厂商服务未运行')
    joint = _check_joint_feedback(suffix, emit=emit)
    if joint:
        problems.append(joint)
    lidar = _check_lidars(suffix, lidar_timeout, emit)
    if lidar:
        problems.append(lidar)
    # Hardware sampling can take seconds: do not reuse a stale green snapshot.
    final_snapshot = resource_snapshot(suffix, active_groups)
    if not final_snapshot.ok:
        problems.insert(0, final_snapshot.detail)
    for detail in problems:
        emit('diagnostic: ' + detail)
    return problems


def check_only(suffix='0_367', active_groups=None, emit=print, lidar_timeout=5.0):
    try:
        problems = _diagnose(suffix, active_groups or {}, emit, lidar_timeout)
        return ('WARN', '；'.join(problems)) if problems else (
            'PASS', '资源归属、服务、关节反馈与双雷达检查通过')
    except Exception as error:
        return 'WARN', f'状态无法确认：{error}'


def _clean_records(active_groups, emit):
    for key in KEYS:
        target = STATE_DIR / (key + '.json')
        if not target.exists():
            continue
        try:
            record = json.loads(target.read_text())
            if (not isinstance(record, dict) or type(record.get('pid')) is not int
                    or type(record.get('start')) is not int):
                raise ValueError('记录字段无效')
        except (OSError, ValueError):
            reason = _quarantine_process_record(key, target)
            if reason and reason.startswith('无法隔离'):
                raise RuntimeError(reason)
            emit(f'repair_action: {key} 无效旧记录已隔离')
            continue
        processes = _processes()
        members = _group(key, record, processes)
        if active_groups.get(key) == record['pid'] and members:
            continue
        if members:
            if not _terminate_group(key, record, emit):
                raise RuntimeError(f'{key} 旧进程组未能退出')
        elif any(p['pgid'] == record['pid'] for p in processes.values()):
            raise RuntimeError(f'{key} 进程组身份变化，未终止')
        target.unlink(missing_ok=True)


def _clear_proven_stale_flag(suffix, snapshot, actions, emit):
    if not actions or any(e.topic == '/map' for e in snapshot.endpoints):
        return
    if any(i.resource != '底盘导航' for i in snapshot.issues):
        return
    _, _, keys = _chassis_state(suffix)
    client = _redis_client()
    try:
        # Never steal a newly acquired token; only clear the legacy flag after
        # the evidenced controller was stopped and all other checks are clean.
        changed = client.eval(
            "if redis.call('EXISTS', KEYS[2]) == 0 and "
            "redis.call('GET', KEYS[1]) == 'true' then "
            "return redis.call('SET', KEYS[1], 'false') end return 0", 2, *keys)
        if changed:
            emit('repair_action: 已停止控制端留下的无持有者导航标志已清理')
    finally:
        client.close()


def run(suffix='0_367', emit=print, lidar_timeout=5.0, takeover=False,
        active_groups=None):
    if not takeover:
        return check_only(suffix, active_groups, emit, lidar_timeout)
    if not _repair.LOCK.acquire(blocking=False):
        return 'WARN', '已有修复或恢复事务正在执行'
    actions = []
    try:
        groups = active_groups or {}
        _clean_records(groups, emit)
        _repair.maintain_guards(emit)
        before = resource_snapshot(suffix, groups)
        emit('repair_scan: ' + before.detail)
        actions, blocked = _repair.execute(before, emit)
        for reason in blocked:
            emit('repair_plan: 待重新确认：' + reason)
        if _gv_control_state() != 'active':
            fixed, reason = _recover_gv_control(suffix, 'start', emit)
            if not fixed:
                return 'WARN', '共享底盘服务未能恢复：' + reason
            actions.append('已恢复共享底盘服务')
        after = resource_snapshot(suffix, groups)
        _clear_proven_stale_flag(suffix, after, actions, emit)
        # Require a continuous clean interval, not the first empty DDS query.
        stable_since, deadline = None, time.monotonic() + 25.0
        emit('repair_verify: 连续观察资源与监管者，至少 15 秒')
        while time.monotonic() < deadline:
            current = resource_snapshot(suffix, groups)
            if current.ok:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 15.0:
                    break
            else:
                stable_since = None
                if any(i.state == 'REOCCUPIED' for i in current.issues):
                    return 'WARN', '监管者重新占用：' + current.detail
            time.sleep(.2)
        else:
            return 'WARN', '资源未稳定移交：' + current.detail
        problems = _diagnose(suffix, groups, emit, lidar_timeout)
        if problems:
            return 'WARN', '；'.join(problems)
        return ('AUTO_RECOVERED' if actions else 'PASS',
                '资源稳定移交并验证通过；' + ('；'.join(actions) or '无需执行停止动作'))
    except Exception as error:
        return 'WARN', str(error)
    finally:
        _repair.LOCK.release()
