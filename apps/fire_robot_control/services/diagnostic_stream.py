"""Rate/size bounded diagnostics with no role in admission or STOP."""
import json
import os
from pathlib import Path
import time


class DiagnosticStream:
    def __init__(self, path, *, interval=1., limit=2*1024*1024, clock=time.monotonic):
        self.path, self.interval, self.limit, self.clock = Path(path), interval, limit, clock
        self.last = None
        self.disabled = False
        self.file = None

    def due(self):
        return not self.disabled and (self.last is None or self.clock()-self.last >= self.interval)

    def emit(self, row):
        if not self.due():
            return
        self.last = self.clock()
        try:
            data = (json.dumps(row, allow_nan=False, separators=(',', ':'))+'\n').encode()
            if len(data) > 65536:
                raise ValueError('DIAGNOSTIC_ROW_TOO_LARGE')
            if self.file is not None and self.file.tell()+len(data) > self.limit:
                self.file.close()
                self.file = None
                os.replace(self.path, self.path.with_suffix(self.path.suffix+'.1'))
            if self.file is None:
                fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
                self.file = os.fdopen(fd, 'wb', buffering=0)
            self.file.write(data)
        except (OSError, ValueError, TypeError):
            self.disabled = True
            self.close()
            try:
                print('APP_DIAGNOSTIC_DISABLED:' + self.path.name, flush=True)
            except OSError:
                # Diagnostic output can disappear during process teardown.
                self.disabled = True

    def close(self):
        if self.file is not None:
            try:
                self.file.close()
            except OSError:
                self.disabled = True
            self.file = None
