"""Shared, read-only resource facts. Endpoint names are never identities."""

from dataclasses import dataclass, field
import os
from pathlib import Path
import re
import shlex
import threading
import time


@dataclass(frozen=True)
class Endpoint:
    topic: str
    name: str
    gid: str


@dataclass(frozen=True)
class Unit:
    name: str
    manager: str
    uid: int
    cgroup: str


@dataclass
class Issue:
    resource: str
    state: str
    detail: str
    owners: list = field(default_factory=list)


@dataclass
class Snapshot:
    issues: list = field(default_factory=list)
    endpoints: list = field(default_factory=list)
    processes: dict = field(default_factory=dict)
    at: float = field(default_factory=time.monotonic)

    @property
    def ok(self):
        return not self.issues

    @property
    def detail(self):
        return "；".join(f"{i.resource}[{i.state}]：{i.detail}" for i in self.issues) or "资源归属检查通过"


def unit_for(process):
    for line in process.get("cgroup", "").splitlines():
        path = line.split(":", 2)[-1]
        parts = path.split("/")
        manager = "user" if any(re.fullmatch(r"user@[0-9]+\.service", p) for p in parts) else "system"
        names = [p for p in parts if p.endswith(".service")
                 and not re.fullmatch(r"user@[0-9]+\.service", p)]
        if names:
            name = names[-1]
            if not re.fullmatch(r"[A-Za-z0-9_.@:-]+\.service", name):
                return None
            return Unit(name, manager, process["uid"], path[:path.rfind(name) + len(name)])
    return None


def node_processes(name, processes):
    base = name.rsplit("/", 1)[-1]
    exact, executable = [], []
    for p in processes.values():
        if p["uid"] != os.getuid():
            continue
        try:
            tokens = p["argv"] if "argv" in p else shlex.split(p["command"])
        except ValueError:
            continue
        if any(t in (f"__node:={base}", f"__node:={name}") for t in tokens):
            exact.append(p)
        elif any(t.rsplit("/", 1)[-1] in (base, base + ".py") for t in tokens):
            executable.append(p)
    return exact or executable


class Graph:
    def __init__(self):
        self.lock = threading.RLock()
        self.context = self.node = self.executor = None

    def read(self, topics, timeout=1.2,
             anchor="/topic_arm_whole_body_and_gripper_current_joints_status_0_367"):
        with self.lock:
            if self.node is None:
                import rclpy
                from rclpy.context import Context
                from rclpy.executors import SingleThreadedExecutor
                self.context = Context()
                rclpy.init(context=self.context)
                self.node = rclpy.create_node("control_center_resource_observer", context=self.context)
                self.executor = SingleThreadedExecutor(context=self.context)
                self.executor.add_node(self.node)
                timeout = max(timeout, 2.5)
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                self.executor.spin_once(timeout_sec=min(.1, max(0, end - time.monotonic())))
            anchors = self.node.get_publishers_info_by_topic(anchor)
            if not anchors or any("UNKNOWN" in x.node_name for x in anchors):
                raise RuntimeError("DDS 硬件参照端尚未发现，不能把空列表判为空闲")
            result = []
            for topic in topics:
                for info in self.node.get_publishers_info_by_topic(topic):
                    result.append(Endpoint(topic,
                        f"{info.node_namespace.rstrip('/')}/{info.node_name}",
                        bytes(info.endpoint_gid).hex()))
            return result

    def close(self):
        with self.lock:
            if self.executor is not None:
                self.executor.shutdown()
            if self.node is not None:
                self.node.destroy_node()
            if self.context is not None:
                self.context.try_shutdown()
            self.context = self.node = self.executor = None


GRAPH = Graph()
LOCAL_TOKENS = frozenset()

def read_process(pid):
    fd = None
    try:
        fd = os.open(Path("/proc") / str(pid), os.O_RDONLY | os.O_DIRECTORY)
        def read(name):
            with os.fdopen(os.open(name, os.O_RDONLY, dir_fd=fd), "rb") as stream:
                return stream.read()
        raw = read("stat").decode(errors="replace")
        fields = raw[raw.rfind(")") + 2:].split()
        argv = [x.decode(errors="replace") for x in read("cmdline").split(b"\0") if x]
        return {"pid": int(pid), "ppid": int(fields[1]), "pgid": int(fields[2]),
                "start": int(fields[19]), "uid": os.fstat(fd).st_uid,
                "cgroup": read("cgroup").decode(errors="replace"),
                "argv": argv, "command": shlex.join(argv)}
    except (OSError, ValueError, IndexError):
        return None
    finally:
        if fd is not None:
            os.close(fd)


def topics_for(suffix):
    return ("/map", f"/topic_gv_target_cmd_vel_{suffix}",
            f"/topic_arm_whole_body_target_joints_position_{suffix}",
            f"/topic_arm_gripper_target_joints_position_{suffix}")


def contracts(suffix):
    # Existing hardware/relay entrypoints can coexist while the shared control
    # lease is idle. This is a role contract, not a programme-specific stop hook.
    return {
        f"/topic_gv_target_cmd_vel_{suffix}": {
            f"/node_arm_vr_control_service_{suffix}", "/dashboard_gv_io",
            f"/s2_chassis_{suffix}", "/cmd_vel_arbiter"},
        f"/topic_arm_whole_body_target_joints_position_{suffix}": {
            f"/node_arm_vr_control_service_{suffix}", "/dashboard_arm_io",
            f"/node_robot_action_service_{suffix}", f"/vision_service_{suffix}",
            f"/action_player_{suffix}", "/whole_body_joint_bridge"},
        f"/topic_arm_gripper_target_joints_position_{suffix}": {
            "/dashboard_arm_io", f"/node_robot_action_service_{suffix}",
            f"/action_player_{suffix}", "/whole_body_joint_bridge"},
    }

def managed_process(process, groups, processes):
    """ROS launch descendants may create their own process groups."""
    if process["pgid"] in groups:
        return True
    seen = set()
    pid = process["pid"]
    while pid in processes and pid not in seen:
        if pid in groups:
            return True
        seen.add(pid)
        pid = processes[pid]["ppid"]
    return False


def same_participant(endpoints):
    """Cyclone DDS RTPS GUIDs contain a 12-byte participant prefix.

    This only groups existing shared-role writers, never map publishers or
    arbitrary controllers. Malformed or unsupported GIDs remain ambiguous.
    """
    return (bool(endpoints)
            and all(re.fullmatch(r"[0-9a-fA-F]{32}", e.gid) for e in endpoints)
            and len({e.gid[:24].lower() for e in endpoints}) == 1)


def inspect_endpoints(endpoints, processes, active_groups, suffix):
    result = Snapshot(endpoints=endpoints, processes=processes)
    by_topic = {topic: [] for topic in topics_for(suffix)}
    for ep in endpoints:
        by_topic.setdefault(ep.topic, []).append(ep)
    groups = set(active_groups.values())
    for topic, eps in by_topic.items():
        unknown = [e for e in eps if not e.gid or "_UNKNOWN_" in e.name or
                   "_NODE_NAME_UNKNOWN_" in e.name or "_NODE_NAMESPACE_UNKNOWN_" in e.name]
        if unknown:
            result.issues.append(Issue(topic, "DISCOVERING", "发布端身份尚未完整，不能判定为空闲"))
            continue
        if topic == "/map":
            owners = []
            for ep in eps:
                owners.extend(p for p in node_processes(ep.name, processes)
                              if not managed_process(p, groups, processes))
            managed = bool(active_groups.get("navigation") or active_groups.get("slam"))
            if len(eps) > 1:
                result.issues.append(Issue(topic, "CONFLICT",
                    f"发现 {len(eps)} 个独立地图发布端（同名也计入）", owners))
            elif eps:
                candidates = node_processes(eps[0].name, processes)
                ours = [p for p in candidates if managed_process(p, groups, processes)]
                if not managed or owners:
                    result.issues.append(Issue(topic, "CONFLICT",
                        f"被外部发布端占用：{eps[0].name}", owners))
                elif not ours:
                    result.issues.append(Issue(topic, "UNKNOWN", "地图存在，但不能证明来自本中心"))
            elif managed:
                result.issues.append(Issue(topic, "DISCOVERING", "本中心模块运行但尚未发现地图输出"))
            continue
        allowed = contracts(suffix).get(topic, set())
        names = {}
        for ep in eps:
            names.setdefault(ep.name, []).append(ep)
        for name, matches in names.items():
            candidates = node_processes(name, processes)
            external = [p for p in candidates if not managed_process(p, groups, processes)]
            # A verified member of a registered controller group is ours even
            # when a Nav2 plugin uses a node name outside the shared-role list.
            # Duplicate endpoints and ambiguous ownership still remain errors.
            if len(matches) == 1 and candidates and not external:
                continue
            shared_role = name in allowed and name not in ("/whole_body_joint_bridge", "/cmd_vel_arbiter")
            duplicate = len(matches) > 1 and not (shared_role and same_participant(matches))
            if name not in allowed or duplicate:
                result.issues.append(Issue(topic, "CONFLICT",
                    f"未授权或重复控制入口：{name}（{len(matches)} 个发布端）", external))
            elif name in ("/whole_body_joint_bridge", "/cmd_vel_arbiter"):
                module = "joint" if name == "/whole_body_joint_bridge" else "navigation"
                ours = [p for p in candidates if managed_process(p, groups, processes)]
                if external or not active_groups.get(module):
                    result.issues.append(Issue(topic, "CONFLICT",
                        f"控制入口 {name} 未由本中心管理", external))
                elif not ours:
                    result.issues.append(Issue(topic, "UNKNOWN", f"不能证明 {name} 来自本中心"))
    return result


def map_process_issues(processes, groups, existing):
    """A live map producer is not free just because its DDS endpoint vanished."""
    known = {p["pid"] for issue in existing if issue.resource == "/map" for p in issue.owners}
    owners = []
    for process in processes.values():
        if (process["uid"] != os.getuid() or managed_process(process, groups, processes)
                or process["pid"] in known):
            continue
        try:
            argv = process.get("argv") or shlex.split(process["command"])
        except ValueError:
            continue
        executable = argv[0] if argv else ""
        if executable.endswith(("/nav2_map_server/map_server",
                                "/cartographer_ros/cartographer_occupancy_grid_node",
                                "/slam_toolbox/async_slam_toolbox_node",
                                "/slam_toolbox/sync_slam_toolbox_node")):
            owners.append(process)
    return [Issue("/map", "CONFLICT",
                  "外部地图服务进程尚未退出，资源不能判为空闲（PID "
                  + ",".join(str(p["pid"]) for p in owners) + "）", owners)] if owners else []


def capture(suffix, active_groups=None, timeout=1.2):
    from . import preflight as p
    groups = {k: int(v) for k, v in (active_groups or {}).items() if k in p.KEYS and v}
    processes = p._processes()
    invalid = []
    starts = {}
    for key, pgid in list(groups.items()):
        leader = processes.get(pgid)
        if not leader or p._group(key, {"pid": pgid, "start": leader["start"]}, processes) is None:
            invalid.append(Issue(key, "UNKNOWN", "本中心进程组身份校验失败"))
            del groups[key]
        else:
            starts[pgid] = leader["start"]
    try:
        endpoints = GRAPH.read(topics_for(suffix), timeout,
            anchor=f"/topic_arm_whole_body_and_gripper_current_joints_status_{suffix}")
        # Refresh process identities after graph discovery; never act on the old
        # start time without revalidation in the executor.
        processes = p._processes()
        for key, pid in list(groups.items()):
            if pid not in processes or processes[pid]["start"] != starts[pid]:
                invalid.append(Issue(key, "UNKNOWN", "本中心启动进程在检查中退出或被复用"))
                del groups[key]
        snapshot = inspect_endpoints(endpoints, processes, groups, suffix)
    except Exception as error:
        snapshot = Snapshot([Issue("ROS", "UNKNOWN", f"发布端查询失败：{error}")], processes=processes)
    snapshot.issues.extend(invalid)
    snapshot.issues.extend(map_process_issues(processes, set(groups.values()), snapshot.issues))
    for key, matches in p._conflicts(processes, suffix).items():
        external = [x for x in matches if not managed_process(x, set(groups.values()), processes)]
        if external:
            snapshot.issues.append(Issue(key, "CONFLICT",
                "发现未经本中心管理的控制进程 PID " + ",".join(str(x["pid"]) for x in external), external))
    try:
        navigating, owner, _ = p._chassis_state(suffix)
        if owner and owner not in LOCAL_TOKENS:
            snapshot.issues.append(Issue("底盘租约", "UNKNOWN", "存在其他控制端持有的租约，不能推定可抢占"))
        elif navigating.lower() == "true" and not owner and not groups.get("navigation"):
            snapshot.issues.append(Issue("底盘导航", "CONFLICT", "导航标志仍被外部控制端占用"))
    except Exception as error:
        snapshot.issues.append(Issue("底盘租约", "UNKNOWN", f"无法确认所有权：{error}"))
    return snapshot
