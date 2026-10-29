"""Optional, bounded evidence for N8R2 USB recovery; no safety state.

The bridge calls record/drain only while holding state_lock. Two latest slots
coalesce bursts; at most eight events are captured per bridge process lifetime.
Create this collector only when diagnostics are enabled. No timer or I/O here.
"""


class SerialRecoveryDiagnostics:
    """Collect BOOT identity and recovery counter advances without a queue."""

    EVENT_LIMIT = 8

    def __init__(self):
        self.captured = 0
        self._boot_epoch = None
        self._recoveries = None
        self._boot = None
        self._usb = None

    def record(self, frame, epoch, receive_mono):
        """Accept validated parser output; never update transport health."""
        if self.captured >= self.EVENT_LIMIT:
            return
        kind = frame['type']
        if kind == 'BOOT':
            if self._boot_epoch == epoch:
                return
            self._boot_epoch = epoch
            event = {'event': 'esp_boot_identity'}
        elif kind == 'USB':
            previous = self._recoveries
            recoveries = frame['recoveries']
            self._recoveries = recoveries
            if recoveries == 0 or (previous is not None and
                                   recoveries <= previous):
                return
            event = {
                'event': 'esp_usb_recovery_event',
                'kind': ('before_first_usb_frame' if previous is None
                         else 'counter_advanced'),
                'previous_recoveries': previous,
            }
        else:
            return
        event.update(epoch=epoch, receive_mono_ns=int(receive_mono * 1e9),
                     frame=frame)
        self.captured += 1
        if kind == 'BOOT':
            self._boot = event
        else:
            self._usb = event

    def drain(self, epoch, session_ready):
        """Return at most two current-session events; consume stale slots too."""
        events = (self._boot, self._usb)
        self._boot = self._usb = None
        if not session_ready:
            return ()
        return tuple(item for item in events
                     if item is not None and item['epoch'] == epoch)
