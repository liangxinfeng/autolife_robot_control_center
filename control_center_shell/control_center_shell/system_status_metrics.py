"""Read-only Linux counters for the control center's system status page."""

from collections import namedtuple
import os
from pathlib import Path
import time

import psutil


Process = namedtuple("Process", "pid name started cpu rss io")
TOP = 30
IO_INTERVAL = 5
LOCAL_FILESYSTEMS = frozenset({"btrfs", "ext2", "ext3", "ext4", "f2fs", "vfat", "xfs"})


def _read_at(fd, name):
    with os.fdopen(os.open(name, os.O_RDONLY, dir_fd=fd), "rb") as stream:
        return stream.read()


def _process(pid, path, with_io, page_size, ticks):
    # Hold the proc directory open so an intervening PID reuse cannot redirect
    # the second read to a different process.
    try:
        fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    except OSError:
        return None
    try:
        raw = _read_at(fd, "stat")
        left, right = raw.index(b"("), raw.rindex(b")")
        name = raw[left + 1:right].decode(errors="replace")
        name = "".join(ch if ch.isprintable() else " " for ch in name)[:96]
        fields = raw[right + 2:].split()  # Field 3 (state) starts at index 0.
        started = int(fields[19])
        cpu = (int(fields[11]) + int(fields[12])) / ticks
        rss = int(fields[21]) * page_size
        io = None
        if with_io:
            try:
                counts = dict(line.split(b":", 1) for line in _read_at(fd, "io").splitlines())
                io = (int(counts[b"read_bytes"]), int(counts[b"write_bytes"]))
            except (OSError, ValueError, KeyError):
                pass
        return Process(pid, name, started, cpu, max(0, rss), io)
    except (OSError, IndexError, ValueError):
        return None
    finally:
        os.close(fd)


def _delta(current, previous):
    if previous is None or any(a < b for a, b in zip(current, previous)):
        return None
    return tuple(a - b for a, b in zip(current, previous))


def _rates(current, previous, elapsed):
    diff = _delta(current, previous)
    return None if diff is None or elapsed <= 0 else tuple(n / elapsed for n in diff)


def _top(rows, metric):
    return sorted(rows, key=lambda row: (-metric(row), row[0], row[1]))[:TOP]


def _filesystems():
    rows, seen = [], set()
    # Only local block filesystems: a disconnected network mount must not
    # turn a status refresh into a blocking filesystem request.
    parts = sorted(psutil.disk_partitions(), key=lambda p: (len(p.mountpoint), p.mountpoint))
    for part in parts:
        if (not part.device.startswith("/dev/") or part.fstype not in LOCAL_FILESYSTEMS
                or part.device in seen):
            continue
        try:
            usage = psutil.disk_usage(part.mountpoint)
        except OSError:
            continue
        seen.add(part.device)
        rows.append((part.mountpoint, part.device, usage.percent, usage.used, usage.free))
    return sorted(rows, key=lambda row: (-row[2], row[0]))


def _disk_counters():
    counters = psutil.disk_io_counters(perdisk=True, nowrap=False) or {}
    return {
        name: (c.read_bytes, c.write_bytes, c.busy_time)
        for name, c in counters.items()
        if (Path("/sys/block") / name / "device").exists() and hasattr(c, "busy_time")
    }


def _net_speeds():
    links = psutil.net_if_stats()
    return {
        name: stats.speed * 1_000_000 / 8
        for name, stats in links.items()
        if stats.isup and stats.speed > 0 and stats.duplex == psutil.NIC_DUPLEX_FULL
        and (Path("/sys/class/net") / name / "device").exists()
        and not (Path("/sys/class/net") / name / "wireless").exists()
    }


def _cpu_fields():
    c = psutil.cpu_times()
    # guest and guest_nice are already included in user/nice.
    values = c._asdict()
    idle = values.get("idle", 0) + values.get("iowait", 0)
    busy = sum(v for k, v in values.items()
               if k not in {"guest", "guest_nice", "idle", "iowait"})
    return busy, idle


class Sampler:
    def __init__(self):
        self.page_size = os.sysconf("SC_PAGE_SIZE")
        self.ticks = os.sysconf("SC_CLK_TCK")
        self.previous = None
        self.io_baseline = None
        self.io_rows = []
        self.io_visible = (0, 0)
        self.io_ready = False

    def sample(self, cancelled):
        now = time.monotonic()
        with_io = self.io_baseline is None or now - self.io_baseline[0] >= IO_INTERVAL
        cpu, mem = _cpu_fields(), psutil.virtual_memory()
        cores = max(1, psutil.cpu_count() or 1)
        disks = _disk_counters()
        net = {n: (c.bytes_recv, c.bytes_sent) for n, c in
               psutil.net_io_counters(pernic=True, nowrap=False).items()}
        procs = {}
        with os.scandir("/proc") as entries:
            for item in entries:
                if cancelled.is_set():
                    return None
                if item.name.isdecimal():
                    proc = _process(int(item.name), item.path, with_io, self.page_size, self.ticks)
                    if proc is not None:
                        procs[(proc.pid, proc.started)] = proc

        last, old_cpu, old_disks, old_net, old_procs = self.previous or (now, None, {}, {}, {})
        elapsed = now - last
        cpu_change = _delta(cpu, old_cpu)
        cpu_pct = None
        if cpu_change and sum(cpu_change) > 0:
            cpu_pct = 100 * cpu_change[0] / sum(cpu_change)

        process_cpu, process_mem = [], []
        for key, proc in procs.items():
            old = old_procs.get(key)
            if old is not None and elapsed > 0 and proc.cpu >= old.cpu:
                use = 100 * (proc.cpu - old.cpu) / (elapsed * cores)
                process_cpu.append((proc.pid, proc.name, min(use, 100.0)))
            process_mem.append((proc.pid, proc.name, proc.rss, 100 * proc.rss / mem.total))
        if with_io:
            current = {key: p.io for key, p in procs.items() if p.io is not None}
            rows = []
            if self.io_baseline is not None:
                then, older = self.io_baseline
                for key, counters in current.items():
                    rates = _rates(counters, older.get(key), now - then)
                    if rates is not None:
                        rows.append((key[0], procs[key].name, *rates))
                self.io_ready = True
            self.io_rows = _top(rows, lambda r: r[2] + r[3])
            self.io_visible = len(current), len(procs)
            self.io_baseline = now, current

        disk_rows = []
        for name, counters in disks.items():
            rates = _rates(counters, old_disks.get(name), elapsed)
            disk_rows.append((name, min(100.0, rates[2] / 10), rates[0], rates[1])
                             if rates is not None else (name, None, None, None))
        disk_rows.sort(key=lambda r: (r[1] is None, -(r[1] or 0), r[0]))

        speeds = _net_speeds()
        net_known, net_other = [], []
        for name, counters in net.items():
            rates = _rates(counters, old_net.get(name), elapsed)
            rx, tx = rates if rates is not None else (None, None)
            speed = speeds.get(name)
            if speed:
                usage = None if rates is None else min(100.0, 100 * max(rates) / speed)
                net_known.append((name, rx, tx, usage))
            else:
                net_other.append((name, rx, tx))
        net_known.sort(key=lambda r: (r[3] is None, -(r[3] or 0), r[0]))
        net_other.sort(key=lambda r: (r[1] is None, -((r[1] or 0) + (r[2] or 0)), r[0]))

        self.previous = now, cpu, disks, net, procs
        return {
            "cpu": cpu_pct, "cores": cores, "memory": (mem.total - mem.available, mem.total),
            "cpu_rows": _top(process_cpu, lambda r: r[2]),
            "memory_rows": _top(process_mem, lambda r: r[2]),
            "filesystems": _filesystems(), "disks": disk_rows,
            "io_rows": self.io_rows, "io_coverage": self.io_visible, "io_ready": self.io_ready,
            "net_known": net_known, "net_other": net_other,
        }
