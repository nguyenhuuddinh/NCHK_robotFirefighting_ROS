"""Validate the bridge's BOOT+live STATE evidence; never use a saved receipt."""
from .operational_contract import FIRMWARE_ELF_SHA256
from .operational_cycle import finite, valid_uuid


IDENTITY_TOPIC='/fire_sensors/serial_session_identity'
# Pi publishes this lightweight snapshot at 5 Hz, independently of STATE.
# Use one shared bound at operator, mapper and all cycle participants. The
# bridge still rejects invalid/unhealthy STATE and changed BOOT/epoch itself.
IDENTITY_MAX_AGE_MS=500.


def identity_key(payload):
    if not isinstance(payload,dict):return None
    instance=payload.get('bridge_instance_id');epoch=payload.get('connection_epoch')
    if (not valid_uuid(instance) or type(epoch) is not int or epoch<1
            or payload.get('elf_sha256')!=FIRMWARE_ELF_SHA256):return None
    return instance,epoch,payload['elf_sha256']


def identity_current(payload,received,now):
    if identity_key(payload) is None or not finite(received) or not finite(now):return False
    age=payload.get('state_age_ms')
    return bool(payload.get('schema_version')==1 and payload.get('frame_protocol')==2
        and payload.get('source')=='SERIAL_V2_BOOT_WITH_LIVE_STATE'
        and payload.get('valid') is True and payload.get('session_ready') is True
        and payload.get('telemetry_healthy') is True and finite(age) and age>=0
        and 0<=now-received<=IDENTITY_MAX_AGE_MS/1000
        and age+(now-received)*1000<=IDENTITY_MAX_AGE_MS
        and type(payload.get('state_esp_ms')) is int and 0<=payload['state_esp_ms']<2**32)


class IdentityLease:
    def __init__(self):
        self.bound=None
        self.fault=None

    def check(self,payload,received,now,*,armed):
        current=identity_current(payload,received,now)
        key=identity_key(payload)
        if self.bound is not None and key is not None and key!=self.bound:
            self.fault=self.fault or 'BOOT_OR_SERIAL_EPOCH_CHANGED'
        if armed and not current:
            self.fault=self.fault or 'BOOT_LIVE_STATE_PROOF_LOST'
        if armed and current and self.bound is None:
            self.bound=key
        return current and self.fault is None


class IdentityContinuity:
    """Stale telemetry holds zero; a different transport/firmware latches fault.

    This does not extend identity_current's shared age bound. Recovery needs
    0.5s of continuous fresh same-connection identity plus stopped/OFF proof.
    It never resets an I budget, creates a publisher, or repeats a pulse.
    """
    HARD_REASONS=frozenset(('MCU_CLOCK_REGRESSION','BOOT_ELF_CHANGED_SAME_CONNECTION',
        'IDENTITY_CLOCK_DISCONTINUITY','IDENTITY_FRAME_INVALID','BOOT_IDENTITY_INVALID'))

    def __init__(self):
        self.bound=None;self.fault=None;self.held=False;self.since=None;self.last_at=None

    def observe(self,payload,received,now,*,armed,stopped):
        current=identity_current(payload,received,now)
        if not finite(now) or self.last_at is not None and now<self.last_at:
            self.fault=self.fault or 'IDENTITY_CLOCK_DISCONTINUITY'
        if self.last_at is not None and finite(now) and now-self.last_at>.35:self.since=None
        self.last_at=now
        if self.bound is not None and isinstance(payload,dict):
            # Delayed wrong/old epoch data is also unsafe to resume from.
            instance=payload.get('bridge_instance_id');epoch=payload.get('connection_epoch');elf=payload.get('elf_sha256')
            if valid_uuid(instance) and type(epoch) is int and epoch>0 and (instance,epoch)!=self.bound[:2]:
                self.fault=self.fault or 'BOOT_OR_SERIAL_EPOCH_CHANGED'
            if isinstance(elf,str) and len(elf)==64 and elf!=self.bound[2]:
                self.fault=self.fault or 'BOOT_OR_SERIAL_EPOCH_CHANGED'
            if payload.get('reason') in self.HARD_REASONS:
                self.fault=self.fault or 'BRIDGE_IDENTITY_FAULT:'+payload['reason']
        if self.fault:return False
        if current and self.bound is None and armed:self.bound=identity_key(payload)
        if armed and not current:self.held=True;self.since=None
        if self.held:
            if not current or stopped is not True:self.since=None;return False
            if self.since is None:self.since=now
            if now-self.since<.5:return False
            self.held=False;self.since=None
        return current
