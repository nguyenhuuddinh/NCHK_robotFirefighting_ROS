"""Read /proc only for the wrapper's own launch descendants, at most 1 Hz."""
import os
from pathlib import Path
import time
from .diagnostic_stream import DiagnosticStream


def process_stat(raw):
    fields = raw[raw.rfind(')')+2:].split()
    return dict(pid=int(raw.split(' ', 1)[0]), pgid=int(fields[2]), sid=int(fields[3]),
                ticks=int(fields[11])+int(fields[12]), threads=int(fields[17]),
                birth=int(fields[19]), rss_pages=int(fields[21]), state=fields[0])


class NavPerformance:
    def __init__(self, path, root_pid, *, proc=Path('/proc'), clock=time.monotonic):
        self.proc, self.root_pid, self.clock = proc, root_pid, clock
        self.group = os.getpgrp()
        self.stream = DiagnosticStream(path, clock=clock)
        self.previous = {}
        self.last = None
        self.hz, self.page_kib = os.sysconf('SC_CLK_TCK'), os.sysconf('SC_PAGE_SIZE')/1024

    def sample(self):
        if not self.stream.due():
            return
        now = self.clock()
        rows, todo, visited = [], [self.root_pid], set()
        while todo and len(visited) < 32:
            pid = todo.pop()
            if pid in visited:
                continue
            visited.add(pid)
            try:
                row = process_stat((self.proc / str(pid) / 'stat').read_text())
                if row['pgid'] != self.group or row['sid'] != self.group:
                    continue
                prior = self.previous.get((pid, row['birth']))
                row['cpu_percent'] = (100*(row['ticks']-prior[0])/self.hz/(now-prior[1])
                                      if prior is not None and now > prior[1] else None)
                self.previous[(pid, row['birth'])] = row['ticks'], now
                row['rss_kib'] = row.pop('rss_pages')*self.page_kib
                rows.append(row)
                children = (self.proc / str(pid) / 'task' / str(pid) / 'children').read_text()
                todo.extend(int(x) for x in children.split())
            except (OSError, ValueError, IndexError):
                continue  # A child can exit between the two read-only snapshots.
        # Keep bounded PID history; never accumulate unrelated/reused PIDs.
        self.previous = {(r['pid'], r['birth']): self.previous[(r['pid'], r['birth'])] for r in rows}
        memory, pressure = {}, {}
        try:
            for line in (self.proc / 'meminfo').read_text().splitlines():
                key, value = line.split(':', 1)
                if key in ('MemAvailable', 'SwapFree', 'SwapTotal'):
                    memory[key+'_kib'] = int(value.split()[0])
            for name in ('cpu', 'memory', 'io'):
                pressure[name] = (self.proc / 'pressure' / name).read_text()[:256]
        except (OSError, ValueError):
            pressure['unavailable'] = True
        self.stream.emit(dict(schema_version=1, scope='PERFORMANCE_ONLY_NOT_READINESS_OR_STOP_PROOF',
            sample_mono=now, wall_ns=time.time_ns(), observer_gap_ms=None if self.last is None else (now-self.last)*1000,
            owned_group=self.group, processes=rows, memory=memory, pressure=pressure))
        self.last = now

    def close(self):
        self.stream.close()
