"""R5 attachment to the supervisor-owned RViz UI; publishes no ROS data."""
import json
from pathlib import Path
import signal
import sys
import time


def wait_view(path, session, stopped, *, clock=time.monotonic, sleep=time.sleep):
    deadline = clock() + 10.
    while clock() < deadline:
        if stopped():
            raise InterruptedError("STOP_BEFORE_VIEW_READY")
        row = json.loads(Path(path).read_text())
        now = clock()
        fresh = 0 <= now - row["sample_mono"] <= .75
        initializing = row.get("view_initializing_until", 0.)
        initializing = type(initializing) in (int, float) and 0 < initializing - now <= 3.
        if row.get("session") != session or not (fresh or initializing):
            raise ValueError("VIEW_FRONTEND_LOST")
        if row.get("view_ready") is True and fresh:
            return
        sleep(.03)
    raise TimeoutError("RVIZ_VIEW_READY_TIMEOUT")


def main(argv=None):
    path, session = (sys.argv[1:] if argv is None else argv)
    stopped = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
    try:
        wait_view(path, session, lambda: stopped[0])
    except InterruptedError:
        return 0
    except (OSError, ValueError, KeyError, TypeError, TimeoutError):
        return 2
    while not stopped[0]:
        try:
            row = json.loads(Path(path).read_text())
            age = time.monotonic() - row["sample_mono"]
            if row["session"] != session or not 0 <= age <= .75:
                return 2
        except (OSError, ValueError, KeyError, TypeError):
            return 2
        time.sleep(.05)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
