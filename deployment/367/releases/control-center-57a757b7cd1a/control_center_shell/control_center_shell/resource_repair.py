"""Bounded supervisor-level handoff. No programme-specific stop hooks."""

import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time
from dataclasses import asdict
from hashlib import sha256

from .resource_observer import Unit, unit_for


def acquire_transaction():
    JOURNAL.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    stream = JOURNAL.with_suffix('.lock').open('a')
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        stream.close()
        return None
    except BaseException:
        stream.close()
        raise
    return stream

PREVIEWED = None
JOURNAL = Path.home() / ".local/state/autolife-control-center/paused-services.json"
UNIT_RE = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.@:-]*\.(service|timer|socket|path)")


def load():
    if not JOURNAL.exists():
        return []
    data = json.loads(JOURNAL.read_text())
    if not isinstance(data, list):
        raise ValueError("暂停服务记录格式无效")
    for entry in data:
        unit = entry.get("unit", {})
        if (not UNIT_RE.fullmatch(unit.get("name", ""))
                or unit.get("manager") not in ("system", "user")
                or unit.get("uid") != os.getuid()):
            raise ValueError("暂停服务记录身份无效")
    return data


def save(entries):
    JOURNAL.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    fd, name = tempfile.mkstemp(dir=JOURNAL.parent, prefix=".paused-")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(entries, f, ensure_ascii=False, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(name, JOURNAL)
    finally:
        Path(name).unlink(missing_ok=True)


def command(unit, action, *args, timeout=45):
    if not UNIT_RE.fullmatch(unit.name) or unit.manager not in ("user", "system"):
        raise ValueError("无效服务身份")
    if unit.uid != os.getuid():
        raise ValueError("不接管其他用户的服务")
    argv = ["/usr/bin/systemctl"]
    if unit.manager == "user":
        argv.append("--user")
    elif action in ("stop", "start", "kill", "reset-failed") and os.geteuid() != 0:
        argv = ["/usr/bin/sudo", "-n"] + argv
    return subprocess.run(argv + [action, unit.name, *args], capture_output=True,
                          text=True, check=False, timeout=timeout)


def properties(unit):
    result = command(unit, "show", "-p", "LoadState", "-p", "ActiveState",
                     "-p", "MainPID", "-p", "ControlGroup", "-p", "TriggeredBy",
                     "-p", "RestartUSec", "-p", "PropagatesStopTo", timeout=4)
    if result.returncode:
        raise RuntimeError((result.stderr or result.stdout).strip())
    props = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    if props.get("LoadState") != "loaded":
        raise RuntimeError(f"{unit.name} 尚未加载")
    return props


def protected(unit):
    from . import preflight as p
    return unit.name in p.NEVER_STOP_UNITS or unit.name.startswith(("user@", "systemd-", "dbus"))

def unit_key(unit):
    return (unit.manager, unit.uid, unit.name) if unit is not None else None


def related_units(unit):
    """Preview the full stop group, including currently active activators."""
    result, visited, visiting = [], set(), set()
    def visit(current):
        key = unit_key(current)
        if key in visiting:
            raise RuntimeError("服务停止传播关系存在环路")
        if key in visited:
            return
        if protected(current):
            raise RuntimeError(f"停止分组包含保护服务 {current.name}")
        visiting.add(key)
        props = properties(current)
        names = props.get("PropagatesStopTo", "").split()
        triggers = props.get("TriggeredBy", "").split()
        for name in dict.fromkeys(names + triggers):
            if not UNIT_RE.fullmatch(name):
                raise RuntimeError("无法确认关联停止单元")
            ref = Unit(name, current.manager, current.uid, "")
            ref_props = properties(ref)
            if name not in names and ref_props.get("ActiveState") not in ("active", "activating"):
                continue
            visit(Unit(name, current.manager, current.uid, ref_props.get("ControlGroup", "")))
        visiting.remove(key)
        visited.add(key)
        result.append(current)
    visit(unit)
    return result


def preview_allows(unit, allowed):
    return any((manager, uid, name) == unit_key(unit) and (not cgroup or cgroup == unit.cgroup)
               for manager, uid, name, cgroup in allowed)

def guard_paths(unit):
    if not UNIT_RE.fullmatch(unit.name) or unit.uid != os.getuid() or unit.manager not in ("user", "system"):
        raise ValueError("无效监管者身份")
    if unit.manager == "user":
        root = Path(f"/run/user/{unit.uid}/systemd/user")
    else:
        root = Path("/run/systemd/system")
    tag = sha256(f"{unit.manager}:{unit.uid}:{unit.name}".encode()).hexdigest()[:20]
    marker = Path(f"/run/user/{unit.uid}/autolife-control-center-handoff") / (tag + ".hold")
    dropin = root / (unit.name + ".d") / "80-control-center-resource-handoff.conf"
    content = "[Unit]\n# Temporary control-center ownership; removed by explicit restore.\n"
    content += f"ConditionPathExists=!{marker}\n"
    return marker, dropin, content


def manager_reload(unit):
    argv = ["/usr/bin/systemctl"]
    if unit.manager == "user":
        argv.append("--user")
    elif os.geteuid() != 0:
        argv = ["/usr/bin/sudo", "-n"] + argv
    result = subprocess.run(argv + ["daemon-reload"], capture_output=True, text=True, timeout=8)
    if result.returncode:
        raise RuntimeError("加载临时接管条件失败：" + (result.stderr or result.stdout).strip())


def handoff_guard(unit, install):
    marker, dropin, content = guard_paths(unit)
    if dropin.exists() and dropin.read_text() != content:
        raise RuntimeError(f"临时接管文件存在其他内容：{dropin}")
    prefix = ["/usr/bin/sudo", "-n"] if unit.manager == "system" and os.geteuid() != 0 else []
    if install:
        marker.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        marker.touch(mode=0o600, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            for argv in ([*prefix, "/usr/bin/install", "-d", "-m", "0755", str(dropin.parent)],
                         [*prefix, "/usr/bin/install", "-m", "0644", stream.name, str(dropin)]):
                result = subprocess.run(argv, capture_output=True, text=True, timeout=8)
                if result.returncode:
                    raise RuntimeError("临时接管条件写入失败：" + (result.stderr or result.stdout).strip())
    else:
        marker.unlink(missing_ok=True)
        if dropin.exists():
            result = subprocess.run([*prefix, "/usr/bin/rm", "--", str(dropin)],
                                    capture_output=True, text=True, timeout=8)
            if result.returncode:
                raise RuntimeError("临时接管条件移除失败：" + (result.stderr or result.stdout).strip())
    manager_reload(unit)


def plan(snapshot):
    """Return only evidenced targets. A missing owner is an unresolved issue."""
    from . import preflight as p
    units, groups, blocked = {}, {}, []
    current = p._processes()
    owner_pids = {owner['pid'] for issue in snapshot.issues for owner in issue.owners}
    owner = p.supervised_launch(current, owner_pids=owner_pids)
    if owner:
        return [], [], [owner]
    for issue in snapshot.issues:
        if issue.state not in ("CONFLICT", "REOCCUPIED") or not issue.owners:
            blocked.append(f"{issue.resource}：{issue.detail}")
            continue
        for owner in issue.owners:
            live = current.get(owner["pid"])
            if live is None:
                continue
            if live["start"] != owner["start"] or live["cgroup"] != owner["cgroup"]:
                blocked.append(f"PID {owner['pid']} 身份已变化")
                continue
            unit = unit_for(live)
            if unit:
                if protected(unit):
                    blocked.append(f"{unit.name} 为共享/硬件保护服务，未停止")
                else:
                    units[(unit.manager, unit.uid, unit.name)] = (unit, live)
            else:
                pgid = live["pgid"]
                leader = current.get(pgid)
                members = [x for x in current.values() if x["pgid"] == pgid]
                safe = (leader and pgid != os.getpgrp() and leader["uid"] == os.getuid()
                        and leader["pid"] == leader["pgid"] and unit_for(leader) is None
                        and all(x["uid"] == os.getuid() and
                                p._descends(x["pid"], pgid, current) for x in members))
                if safe:
                    groups[pgid] = leader
                else:
                    blocked.append(f"PID {live['pid']} 的独立进程组边界无法证明")
    return list(units.values()), list(groups.values()), list(dict.fromkeys(blocked))


def stop_unit(unit, evidence, emit, *, parent=None, expanded=False):
    from . import preflight as p
    if protected(unit):
        raise RuntimeError(f"{unit.name} 为保护服务")
    props = properties(unit)
    members = [x for x in p._processes().values() if unit_key(unit_for(x)) == unit_key(unit)]
    prior_active = props.get("ActiveState") not in ("inactive", "failed") or bool(members)
    live = p._proc(evidence["pid"]) if evidence else None
    if props.get("ControlGroup") != unit.cgroup and prior_active:
        raise RuntimeError(f"{unit.name} cgroup 已变，重新自检")
    if evidence and (not live or live["start"] != evidence["start"] or unit_for(live) != unit):
        emit(f"repair_note: {unit.name} 子进程变化，按已确认的同一监管单元接管")
    if not expanded:
        linked = related_units(unit)
        if PREVIEWED is not None and any(not preview_allows(u, PREVIEWED[1]) for u in linked):
            raise RuntimeError("服务分组在确认后发生变化，请重新确认")
        for child in linked[:-1]:
            stop_unit(child, None, emit, parent=unit.name, expanded=True)

    entries = load()
    key = (unit.name, unit.manager)
    if not any((e["unit"]["name"], e["unit"]["manager"]) == key for e in entries):
        entries.append({"unit": asdict(unit), "state": "stopping",
                        "guard_only": not prior_active, "parent": parent,
                        "at": time.time(), "prior_main_pid": props.get("MainPID", "")})
    for entry in entries:
        if (entry["unit"]["name"], entry["unit"]["manager"]) == key:
            entry["state"] = "stopping"
    save(entries)
    handoff_guard(unit, True)
    emit(f"repair_plan: stop {unit.manager}/{unit.name} pid={props.get('MainPID')} cgroup={unit.cgroup}")
    result = command(unit, "stop", "--no-block")
    if result.returncode:
        raise RuntimeError(f"{unit.name} 停止失败：{(result.stderr or result.stdout).strip()}")
    if not wait_unit_quiet(unit, 8.0):
        # Force only the already-confirmed service cgroup, never a node-name
        # search or a global killall. Hardware units remain protected above.
        after = properties(unit)
        if after.get("ControlGroup") != unit.cgroup:
            raise RuntimeError(f"{unit.name} cgroup 已变化，未强制终止")
        emit(f"repair_force: {unit.name} 正常停止超时，向已确认单元全部进程发送 SIGKILL")
        result = command(unit, "kill", "--kill-whom=all", "--signal=SIGKILL")
        if result.returncode:
            raise RuntimeError(f"{unit.name} 强制终止失败：{(result.stderr or result.stdout).strip()}")
        if not wait_unit_quiet(unit, 5.0):
            raise RuntimeError(f"{unit.name} 强制终止后仍未完全退出")
    if properties(unit).get("ActiveState") == "failed":
        reset = command(unit, "reset-failed")
        if reset.returncode:
            emit(f"repair_note: {unit.name} 历史失败标记未清除，进程已退出")
    entries = load()
    for entry in entries:
        if (entry["unit"]["name"], entry["unit"]["manager"]) == key:
            entry["state"] = "paused"
    save(entries)
    emit(f"repair_action: {unit.name} 监管者及子进程已暂停（可手动恢复）")


def wait_unit_quiet(unit, timeout):
    from . import preflight as p
    deadline = time.monotonic() + timeout
    while True:
        state = properties(unit).get("ActiveState")
        live = any(unit_key(unit_for(x)) == unit_key(unit) for x in p._processes().values())
        if state in ("inactive", "failed") and not live:
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(.2)


def stop_group(identity, emit):
    from . import preflight as p
    pgid = identity["pid"]
    current = p._processes()
    live = current.get(pgid)
    if not live:
        return
    if live["start"] != identity["start"] or live["pgid"] != pgid or unit_for(live):
        raise RuntimeError(f"进程组 {pgid} 身份已变")
    for member in current.values():
        if member["pgid"] == pgid and (member["uid"] != os.getuid() or
                                      not p._descends(member["pid"], pgid, current)):
            raise RuntimeError("进程组中有未确认归属成员")
    for signum, limit in ((signal.SIGINT, 2), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
        live = p._proc(pgid)
        if live is not None and live["start"] != identity["start"]:
            raise RuntimeError(f"进程组 {pgid} PID 已复用")
        if not any(x["pgid"] == pgid for x in p._processes().values()):
            return
        try:
            os.killpg(pgid, signum)
        except ProcessLookupError:
            return
        except PermissionError as error:
            raise RuntimeError(f"无权终止进程组 {pgid}") from error
        end = time.monotonic() + limit
        while time.monotonic() < end:
            if not any(x["pgid"] == pgid for x in p._processes().values()):
                emit(f"repair_action: 独立进程组 {pgid} 已退出")
                return
            time.sleep(.1)
    raise RuntimeError(f"进程组 {pgid} 未能退出")


def paused_issues():
    from .resource_observer import Issue
    errors = []
    for entry in load():
        unit = Unit(**entry["unit"])
        try:
            state = properties(unit).get("ActiveState")
            if state not in ("inactive", "failed"):
                from . import preflight as p
                owners = [x for x in p._processes().values() if unit_key(unit_for(x)) == unit_key(unit)]
                state = "REOCCUPIED" if entry.get("state") == "paused" else "CONFLICT"
                detail = "本次暂停的监管者已重新运行" if state == "REOCCUPIED" else "上次停止操作未完成"
                errors.append(Issue(unit.name, state, detail, owners))
            else:
                marker, dropin, content = guard_paths(unit)
                if not marker.exists() or not dropin.exists() or dropin.read_text() != content:
                    errors.append(Issue(unit.name, "UNKNOWN", "临时接管条件缺失，请重新修复"))
        except Exception as error:
            errors.append(Issue(unit.name, "UNKNOWN", f"无法验证暂停状态：{error}"))
    return errors

def maintain_guards(emit):
    for entry in load():
        unit = Unit(**entry["unit"])
        if entry.get("state") == "paused" and properties(unit).get("ActiveState") in ("inactive", "failed"):
            marker, dropin, content = guard_paths(unit)
            if not marker.exists() or not dropin.exists() or dropin.read_text() != content:
                if protected(unit):
                    raise RuntimeError("暂停记录指向保护服务，未改动启动条件")
                handoff_guard(unit, True)
                emit(f"repair_action: {unit.name} 临时接管条件已补齐")


def execute(snapshot, emit):
    units, groups, blocked = plan(snapshot)
    if PREVIEWED is None:
        raise RuntimeError("请先预览并确认本次修复计划")
    at, allowed_units, allowed_groups = PREVIEWED
    if time.monotonic() - at > 120:
        raise RuntimeError("修复计划已过期，请重新确认")
    if any(not preview_allows(u, allowed_units) for u, _ in units):
        raise RuntimeError("冲突服务身份或计划发生变化，请重新自检并确认")
    if any((p["pid"], p["start"]) not in allowed_groups for p in groups):
        raise RuntimeError("冲突进程组发生变化，请重新自检并确认")
    for unit, evidence in units:
        stop_unit(unit, evidence, emit)
    for identity in groups:
        stop_group(identity, emit)
    return [f"已暂停 {u.name}" for u, _ in units] + [
        f"已清理独立进程组 {x['pid']}" for x in groups], blocked


def restore(emit, suffix='0_367'):
    from . import preflight as p
    transaction = acquire_transaction()
    if transaction is None:
        return "WARN", "其他修复/恢复正在运行"
    try:
        entries, failures = load(), []
        if not entries:
            return "PASS", "没有需要恢复的暂停服务"
        if any(p._conflicts(p._processes(), suffix).values()):
            return "WARN", "请先停止本中心及其他正在运行的导航/建图/关节控制模块"
        snapshot = p.resource_snapshot(suffix)
        if not snapshot.ok:
            return "WARN", "资源尚被占用或归属未确认，未恢复任何服务：" + snapshot.detail
        if any(protected(Unit(**e["unit"])) for e in entries):
            return "WARN", "暂停记录包含保护服务，未恢复任何服务"
        failed_parents = set()
        # Services first; activation sources last.
        ordered = sorted(entries, key=lambda e: (
            not e["unit"]["name"].endswith(".service"), bool(e.get("parent"))))
        for entry in ordered:
            unit = Unit(**entry["unit"])
            if (unit.manager, entry.get("parent")) in failed_parents:
                failed_parents.add((unit.manager, unit.name))
                failures.append(f"{unit.name}：主服务恢复失败，保留暂停条件")
                continue
            try:
                if protected(unit):
                    raise RuntimeError("记录指向保护服务，拒绝恢复")
                props = properties(unit)
                if entry.get("guard_only"):
                    handoff_guard(unit, False)
                    entries.remove(entry)
                    save(entries)
                    emit(f"restore: {unit.name} 已解除临时限制（原先未运行，未主动启动）")
                    continue
                if props.get("ActiveState") not in ("active", "activating"):
                    handoff_guard(unit, False)
                    if props.get("ActiveState") == "failed":
                        reset = command(unit, "reset-failed")
                        if reset.returncode and "not loaded" not in (reset.stderr or ""):
                            raise RuntimeError((reset.stderr or reset.stdout).strip())
                    result = command(unit, "start")
                    if result.returncode:
                        raise RuntimeError((result.stderr or result.stdout).strip())
                else:
                    handoff_guard(unit, False)
                state = properties(unit).get("ActiveState")
                deadline = time.monotonic() + 4.0
                while state == "activating" and time.monotonic() < deadline:
                    time.sleep(.2)
                    state = properties(unit).get("ActiveState")
                if state != "active":
                    raise RuntimeError(f"服务未进入运行状态（{state}）")
                entries.remove(entry)
                save(entries)
                emit(f"restore: {unit.manager}/{unit.name} 已恢复")
            except Exception as error:
                failed_parents.add((unit.manager, unit.name))
                try:
                    handoff_guard(unit, True)
                except Exception as guard_error:
                    emit(f"restore: 保持临时接管条件失败：{guard_error}")
                failures.append(f"{unit.name}：{error}")
        return ("WARN", "；".join(failures)) if failures else (
            "PASS", "本次暂停服务已恢复；原导航目标未由控制中心恢复，请重新自检")
    finally:
        transaction.close()
