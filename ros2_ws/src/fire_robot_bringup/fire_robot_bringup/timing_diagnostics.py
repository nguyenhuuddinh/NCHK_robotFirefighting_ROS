"""Windowed timing observations, independent of admission and safety decisions."""

import threading
import time


class TimingDiagnostics:
    """Keep bounded statistics for caller-supplied, fixed metric names.

    Event gaps use monotonic time. Signed observations (such as header age)
    retain negative values. A 300 ms exceedance is diagnostic only. Snapshots
    clear window statistics but retain event timestamps across window edges.
    """

    def __init__(self, enabled=True):
        self.enabled = bool(enabled)
        self._lock = threading.Lock() if self.enabled else None
        self._last = {} if self.enabled else None
        self._values = {} if self.enabled else None

    def _observe_locked(self, name, milliseconds):
        values = self._values
        count = name + '_count'
        minimum = name + '_min_ms'
        maximum = name + '_max_ms'
        over = name + '_over_300ms'
        values[count] = values.get(count, 0) + 1
        values[minimum] = min(values.get(minimum, milliseconds), milliseconds)
        values[maximum] = max(values.get(maximum, milliseconds), milliseconds)
        values[over] = values.get(over, 0) + int(milliseconds > 300.0)

    def observe(self, name, milliseconds):
        """Record a duration or signed age without controlling runtime behavior."""
        if not self.enabled:
            return
        with self._lock:
            self._observe_locked(name, milliseconds)

    def mark(self, name, now=None):
        """Record an event and the gap from its previous occurrence, if any."""
        if not self.enabled:
            return
        if now is None:
            now = time.monotonic()
        with self._lock:
            previous = self._last.get(name)
            self._last[name] = now
            count = name + '_events'
            self._values[count] = self._values.get(count, 0) + 1
            if previous is not None:
                self._observe_locked(name + '_gap', (now - previous) * 1000.0)

    def increment(self, name):
        """Count an event separately from timing observations."""
        if not self.enabled:
            return
        with self._lock:
            self._values[name] = self._values.get(name, 0) + 1

    def snapshot(self):
        """Consume window statistics; an absent metric means no observations."""
        if not self.enabled:
            return {}
        with self._lock:
            values = self._values
            self._values = {}
        return values

    def format_snapshot(self):
        """Consume and format one window for an existing low-rate log call."""
        return ' '.join(
            f'{key}={value:.3f}' if isinstance(value, float) else f'{key}={value}'
            for key, value in sorted(self.snapshot().items()))
