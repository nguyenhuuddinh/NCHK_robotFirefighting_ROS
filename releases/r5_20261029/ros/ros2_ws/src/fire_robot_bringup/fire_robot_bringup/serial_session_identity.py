"""Read-only BOOT/connection identity, kept separate from bridge health.

Call under the bridge state_lock. No serial I/O or changes to readiness,
watchdog, command queue, reconnect or transport ownership are made here.
"""
import math
import uuid


IDENTITY_TOPIC = '/fire_sensors/serial_session_identity'


def uint32(value):
    return type(value) is int and 0 <= value < 2**32


def digest(value):
    return isinstance(value,str) and len(value)==64 and all(c in '0123456789abcdef' for c in value)


class SerialSessionIdentity:
    def __init__(self, instance_id=None):
        self.instance = instance_id or uuid.uuid4().hex
        self.epoch = None
        self.boot = self.state = None
        self.state_at = None
        self.fault = None
        self.last_at = None

    def _epoch(self, epoch):
        if epoch != self.epoch:
            self.epoch = epoch
            self.boot = self.state = None
            self.state_at = None
            self.fault = None
            self.last_at = None

    def observe(self, frame, epoch, now):
        self._epoch(epoch)
        if not math.isfinite(now) or (self.last_at is not None and now < self.last_at):
            self.fault = 'IDENTITY_CLOCK_DISCONTINUITY'
            return
        self.last_at = now
        if not isinstance(frame,dict) or frame.get('type') not in ('BOOT','STATE'):
            return
        if not uint32(frame.get('esp_ms')) or not uint32(frame.get('seq')):
            self.fault = 'IDENTITY_FRAME_INVALID'; return
        if frame['type']=='BOOT':
            if not digest(frame.get('elf_sha256')) or not uint32(frame.get('reset_reason')):
                self.fault = 'BOOT_IDENTITY_INVALID'; return
            if self.boot is not None and frame['elf_sha256'] != self.boot['elf_sha256']:
                self.fault = 'BOOT_ELF_CHANGED_SAME_CONNECTION'; return
            if self.state and (frame['esp_ms']-self.state['esp_ms']) % 2**32 >= 2**31:
                self.fault = 'MCU_CLOCK_REGRESSION'; return
            self.boot = dict(frame)
        else:
            if self.state:
                delta=(frame['esp_ms']-self.state['esp_ms']) % 2**32
                if delta >= 2**31:
                    self.fault = 'MCU_CLOCK_REGRESSION'
                if delta == 0:
                    return  # A repeated MCU timestamp never renews freshness.
            self.state = dict(frame)
            self.state_at = now

    def snapshot(self, epoch, now, ready, healthy):
        self._epoch(epoch)
        age=(now-self.state_at)*1000 if self.state_at is not None and math.isfinite(now) else None
        forward=(self.boot is not None and self.state is not None
                 and (self.state['esp_ms']-self.boot['esp_ms']) % 2**32 < 2**31)
        valid=(type(epoch) is int and epoch>0 and ready is True and healthy is True
               and self.fault is None and forward and age is not None and 0<=age<=200.)
        if self.fault:
            reason=self.fault
        elif self.boot is None:
            reason='WAIT_BOOT'
        elif not forward or age is None or not 0<=age<=200.:
            reason='WAIT_FRESH_STATE'
        elif not ready or not healthy:
            reason='BRIDGE_NOT_HEALTHY'
        else:
            reason='BOOT_AND_STATE_CURRENT_CONNECTION'
        return dict(schema_version=1, source='SERIAL_V2_BOOT_WITH_LIVE_STATE',
            frame_protocol=2, bridge_instance_id=self.instance, connection_epoch=epoch,
            valid=bool(valid), reason=reason, session_ready=ready is True,
            telemetry_healthy=healthy is True,
            elf_sha256=self.boot['elf_sha256'] if self.boot else None,
            boot_esp_ms=self.boot['esp_ms'] if self.boot else None,
            boot_seq=self.boot['seq'] if self.boot else None,
            state_esp_ms=self.state['esp_ms'] if self.state else None,
            state_seq=self.state['seq'] if self.state else None,
            state_age_ms=round(age,3) if age is not None else None,
            cryptographic_attestation=False, updates_bridge_health=False)
