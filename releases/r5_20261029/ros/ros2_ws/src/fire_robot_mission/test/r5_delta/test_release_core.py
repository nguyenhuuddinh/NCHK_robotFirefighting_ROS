"""New retirement protocol; actual coordinator + all participant state machines."""
import copy
import pytest
from fire_robot_mission.cycle_release import (
    CycleReleaseCoordinator,CycleReleaseParticipant,ReleaseContext,ROLES)
from fire_robot_mission.operational_identity import IdentityLease
from fire_robot_mission.operational_contract import FIRMWARE_ELF_SHA256 as ELF
from fire_robot_bringup.serial_session_identity import SerialSessionIdentity
from fire_robot_mission.ir_range_core import IrRangePreviewCore
from test_installed_pulse import ready

OLD='a'*32;CYCLE='cd'*16;TOKEN='ef'*16;BRIDGE='ab'*16


class Chain:
    def __init__(self,*,no_goal=False,after_pulse=False):
        self.now=1.;self.seq=10;self.after=after_pulse
        self.context=dict(cycle_id=CYCLE,nav_goal_uuid=None if no_goal else OLD,
            bridge_instance_id=BRIDGE,connection_epoch=1,elf_sha256=ELF)
        self.master=CycleReleaseCoordinator(lambda:TOKEN)
        self.master.begin(self.context,self.now,after_pulse=after_pulse,home_before=0 if after_pulse else None)
        self.peers={role:CycleReleaseParticipant(role) for role in ROLES}
        self.resets={role:0 for role in ROLES}
        self.identity=SerialSessionIdentity(BRIDGE)
        self.range=IrRangePreviewCore(baselines=(26060,26140,25825),baseline_tolerance=500,min_active_drop=1000,
            right_thresholds=(800,1000,20000),median_window=5,stable_samples=3,
            ir_timeout_s=.6,vision_timeout_s=.5,max_ir_age_ms=300.,max_image_age_ms=300.,
            spray_confirm_samples=5,spray_reentry_margin_counts=0,range_policy='YOLO_RIGHT_ADC_20261025')
        self.identity.observe(dict(type='BOOT',seq=1,esp_ms=1,reset_reason=1,elf_sha256=ELF),1,1.)
        for peer in self.peers.values():
            assert peer.receive(dict(protocol='CYCLE_RELEASE_V1',phase='ACTIVE',context=self.context),1.)

    def tick(self,*,fault=None,skip_ack=None,skip_peer=None,mutate=None):
        self.now+=.1;self.seq+=1;now=self.now
        d=ready();uid=self.context['nav_goal_uuid']
        d.update(cmd=(0.,)*6,odom=(0.,)*6)
        d['status']=[(uid,5)] if uid else []
        d['guard'].update(goal_uuid=uid)
        self.identity.observe(dict(type='STATE',seq=self.seq,esp_ms=round(now*1000)),1,now)
        d['identity']=self.identity.snapshot(1,now,True,True)
        d['vision']['fire']=dict(detected_now=False,confirmed=False)
        self.range.update_detection(d['vision'],now)
        self.range.update_alignment(dict(schema_version=1,observe_only=True,state='NO_TARGET',
            safety=dict(motion_command_publisher=False,actuator_command_publisher=False)),now)
        self.range.update_ir(dict(schema_version=1,valid=True,left=26000,center=26000,right=26000,
                                  sample_age_ms=10.,receive_age_ms=10.),now)
        d['ir']=self.range.snapshot(now)
        d['actor']=dict(state='DONE' if self.after else 'WAIT_P',goal_uuid=uid,
            cycle_id=self.context['cycle_id'],r5_cycle_context=self.context,start_home_count=0,
            request_used=self.after,pump_on_feedback_seen=self.after,servo_sweep_feedback_seen=self.after,
            proposed_pump_on=False)
        d['review']=dict(schema_version=1,mode='SHADOW_ONLY',state='NO_FIRE_EVIDENCE',
            goal_uuid=uid,pulse_index=1,blockers=[],cycle_context=self.context,
            scope='R5_CANONICAL_REVIEW_FROM_ACTUAL_PULSE_CORE')
        d['actuator']['home_command_completed_count']=1 if self.after else 0
        graph=True;raw=pump=0
        if fault=='graph':graph=False
        if fault=='raw':raw=1
        if fault=='pump_owner':pump=1
        if fault=='moving':d['odom']=[.03,0,0,0,0,0]
        if fault=='cmd':d['cmd']=[.01,0,0,0,0,0]
        if fault=='ACT_ON':d['actuator']['pump_pwm_commanded_on']=True
        if fault=='LOCKED':d['actuator']['servo_phase']='LOCKED'
        if fault=='ACT_stale':d['actuator']['receive_age_ms']=351
        if fault=='boot_epoch':d['identity']['connection_epoch']=2
        if fault=='boot_stale':d['identity']['state_age_ms']=501
        if fault=='active_goal':d['status']=[('12'*16,2)]+d['status']
        if fault=='not_canceled' and uid:d['status']=[(uid,6)]
        if fault=='yolo_fire':d['vision']['fire']=dict(detected_now=True,confirmed=True)
        if fault=='yolo_stale':d['vision']['source']['age_ms']=301
        if fault=='IR_ambiguous':d['ir']['filtered_counts']['right']=19999
        if fault=='IR_stale':d['ir']['ir_receive_age_ms']=501
        if fault=='no_home':d['actuator']['home_command_completed_count']=0
        if fault=='no_ON_ACK':d['actor']['pump_on_feedback_seen']=False
        if fault=='no_review':d['review']['state']='REVIEWING'
        if fault=='IR_bad_blocker':d['ir']['blockers']=[{}]
        if fault=='vision_bad_source':d['vision']['source']=[]
        if mutate:mutate(d)
        control=self.master.control()
        reports={}
        for role,peer in self.peers.items():
            if role==skip_peer:continue
            assert peer.receive(control,now)
            for key,value in d.items():peer.update(key,value,now)
            def reset(ctx,r=role):
                assert ctx.as_dict()==self.context
                self.resets[r]+=1
            ack=peer.step(now,graph_ok=graph,raw_count=raw,pump_count=pump,reset=reset)
            report=copy.deepcopy(d.get(role,{}));report['cycle_release_ack']=ack
            if role=='guard' and ack['phase']=='COMMITTED':
                report.update(goal_uuid=None,state='WAITING_FOR_INTENT')
            reports[role]=report
        for key,value in d.items():self.master.update(key,value,now)
        for role,value in reports.items():
            if role!=skip_ack:self.master.update(role,value,now)
        return self.master.step(now,graph_ok=graph,raw_count=raw,pump_count=pump)

    def finish(self):
        for _ in range(60):
            report=self.tick()
            if report['ready']:return report
        raise AssertionError(report)


@pytest.mark.parametrize('no_goal,after',[(False,False),(True,False),(False,True),(True,True)])
def test_two_phase_release_waits_for_actual_peer_reset_and_keeps_optional_goal(no_goal,after):
    chain=Chain(no_goal=no_goal,after_pulse=after)
    report=chain.finish()
    assert report['ready'] and chain.master.receipt.context.nav_goal_uuid==(None if no_goal else OLD)
    assert set(chain.resets.values())=={1}
    for _ in range(5):assert chain.tick()['ready']
    assert set(chain.resets.values())=={1} and not report['old_goal_resumed']


@pytest.mark.parametrize('fault',['graph','raw','pump_owner','moving','cmd','ACT_ON','LOCKED','ACT_stale',
    'boot_epoch','boot_stale','active_goal','not_canceled','yolo_fire','yolo_stale','IR_ambiguous','IR_stale',
    'IR_bad_blocker','vision_bad_source'])
def test_each_missing_proof_holds_without_fake_READY_or_exception(fault):
    chain=Chain()
    for _ in range(30):report=chain.tick(fault=fault)
    assert not report['ready'] and report['blockers'] and chain.master.receipt is None


@pytest.mark.parametrize('fault',['no_home','no_ON_ACK','no_review'])
def test_postspray_cannot_reuse_prespray_release_without_pulse_OFF_home_review(fault):
    chain=Chain(after_pulse=True)
    for _ in range(30):report=chain.tick(fault=fault)
    assert not report['ready'] and report['blockers']


def test_missing_peer_ACK_cannot_be_replaced_by_five_other_ACKs():
    chain=Chain()
    for _ in range(30):report=chain.tick(skip_ack='ir_live')
    assert report['phase']=='PREPARE' and 'WAIT_IR_LIVE_PREPARED' in report['blockers']
    assert chain.finish()['ready']


def test_late_guard_commit_and_pruned_status_do_not_drop_cycle_or_invent_cancel():
    chain=Chain()
    for _ in range(25):
        report=chain.tick(skip_peer='guard' if chain.master.phase=='COMMIT' else None)
        if chain.master.phase=='COMMIT':break
    assert chain.master.phase=='COMMIT'
    for _ in range(5):assert not chain.tick(skip_peer='guard')['ready']
    # Old cancellation was actually observed. Status may now prune the UUID;
    # participant reset ACKs still bind the same token, not a fake status5.
    for _ in range(30):
        report=chain.tick(mutate=lambda d:d.update(status=[]))
        if report['ready']:break
    assert report['ready'] and chain.master.context.nav_goal_uuid==OLD


def test_clear_lost_during_COMMIT_needs_another_full_fresh_hold():
    chain=Chain()
    for _ in range(40):
        chain.tick()
        if chain.master.phase=='COMMIT':break
    assert chain.master.phase=='COMMIT'
    chain.tick(fault='yolo_fire')
    for _ in range(5):assert not chain.tick()['ready']
    assert chain.finish()['ready']


def test_commit_without_local_reset_never_ACKs_COMMITTED():
    chain=Chain();peer=chain.peers['actor']
    for _ in range(10):chain.tick()
    msg=chain.master.control();msg['phase']='COMMIT'
    peer.receive(msg,chain.now)
    ack=peer.step(chain.now,graph_ok=True,raw_count=0,pump_count=0,reset=None)
    assert ack['phase']!='COMMITTED' and ack['blockers']==['WAIT_LOCAL_RESET']


def test_context_or_token_change_and_ACTIVE_downgrade_are_rejected():
    chain=Chain();chain.tick();peer=chain.peers['ir_live']
    message=chain.master.control();message['request_id']='12'*16
    assert not peer.receive(message,chain.now)
    peer2=chain.peers['alignment'];message=chain.master.control();message['phase']='ACTIVE'
    assert not peer2.receive(message,chain.now)
    assert peer2.error=='RELEASE_DOWNGRADE_TO_ACTIVE'


def test_identity_lease_binds_runtime_epoch_and_does_not_rearm_after_reconnect():
    chain=Chain();chain.tick();payload=chain.master.inputs['identity'][0]
    lease=IdentityLease();assert lease.check(payload,chain.now,chain.now,armed=True)
    changed=dict(payload,connection_epoch=2)
    assert not lease.check(changed,chain.now,chain.now,armed=True)
    assert lease.fault=='BOOT_OR_SERIAL_EPOCH_CHANGED'
    assert not lease.check(payload,chain.now,chain.now,armed=False)
