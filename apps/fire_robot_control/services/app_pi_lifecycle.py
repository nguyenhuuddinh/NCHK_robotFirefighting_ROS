"""App-only prearm failure drain; R5's PiController file is never edited."""
from r5_pi_session import PiController
import signal

DRAIN_SECONDS = 30.


class AppStopSignal:
    """A terminal hangup must enter the same STOP/drain path as SIGINT."""
    def __init__(self, stop):
        self.stop = stop
        self.reason = None

    def __call__(self, number, _frame):
        self.stop[0] = True
        if self.reason is None:
            self.reason = signal.Signals(number).name


class AppPiController(PiController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.failure_drain_at = None

    def _try_seal(self, now):
        if (self.phase == "IO_STARTING" and self.seal_requested_at is not None
                and now - self.seal_requested_at >= 20.):
            self.error = "PI_SEAL_HEALTH_TIMEOUT"
            self.failure_drain_at = now
            self.phase = "WAIT_LAPTOP_DRAIN"
        else:
            super()._try_seal(now)

    def command(self, cmd, now):
        if self.phase == "WAIT_LAPTOP_DRAIN" and cmd not in ("STATE", "STOP"):
            raise ValueError("APP_PREARM_FAILED_STOP_ONLY")
        return super().command(cmd, now)

    def tick(self, now):
        if self.phase == "WAIT_LAPTOP_DRAIN" and now - self.failure_drain_at >= DRAIN_SECONDS:
            super().command("STOP", now)
        return super().tick(now)

    def report(self):
        row = super().report()
        if self.phase == "WAIT_LAPTOP_DRAIN":
            row["health"] = self.health()
            row["drain_remaining_s"] = max(0., DRAIN_SECONDS - (self.last - self.failure_drain_at))
        row["app_prearm_drain"] = True
        row["app_stop_signal"] = getattr(getattr(self, "stop_signal", None), "reason", None)
        return row
