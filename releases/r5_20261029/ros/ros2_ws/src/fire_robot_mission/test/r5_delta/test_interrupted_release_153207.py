"""Interrupted unused-cycle release: fresh STOP remains mandatory."""
import pytest
from test_release_core import Chain
from fire_robot_mission.cycle_release import CycleReleaseCoordinator


def interrupted():
    c=Chain();c.master=CycleReleaseCoordinator(lambda:'ef'*16)
    c.master.begin(c.context,c.now,after_pulse=False,home_before=None,require_clear=False)
    return c


def test_interrupted_release_ignores_fire_but_not_STOP():
    c=interrupted()
    for _ in range(60):
        r=c.tick(fault='yolo_fire')
        if r['ready']:break
    assert r['ready'] and set(c.resets.values())=={1}


@pytest.mark.parametrize('fault',['moving','cmd','ACT_ON','ACT_stale','boot_epoch','raw','pump_owner','active_goal','not_canceled'])
def test_interrupted_release_keeps_each_physical_STOP_gate(fault):
    c=interrupted()
    for _ in range(40):r=c.tick(fault=fault)
    assert not r['ready'] and c.master.receipt is None and r['blockers']


def test_clear_exemption_forbidden_after_pulse():
    c=Chain()
    coordinator=CycleReleaseCoordinator()
    with pytest.raises(ValueError):
        coordinator.begin(c.context,c.now,after_pulse=True,home_before=0,require_clear=False)
    assert coordinator.context is None and coordinator.control() is None
    peer=c.peers['actor'];msg=c.master.control();msg.update(after_pulse=True,home_before=0,require_clear=False)
    assert not peer.receive(msg,c.now) and peer.error=='RELEASE_POLICY_INVALID'


def test_clear_policy_cannot_change_during_release():
    c=interrupted();c.tick();msg=c.master.control();msg['require_clear']=True
    assert not c.peers['actor'].receive(msg,c.now)
    assert c.peers['actor'].error=='RELEASE_POLICY_CHANGED'


@pytest.mark.parametrize('bad',['no_historical_cancel','wrong_uuid','wrong_terminal','bad_receipt'])
def test_downstream_failure_never_invents_exact_cancel(bad):
    c=interrupted();c.tick()
    def mutate(d):
        d['guard'].update(state='FAILED',blockers=['IR_APPROACH_CANCEL_PROOF_LOST'])
        if bad=='wrong_uuid':d['guard']['goal_uuid']='56'*16
        if bad=='wrong_terminal':d['guard']['goal_status']=6
        if bad=='bad_receipt':d['guard']['cancel_service_accepted']=False
    if bad=='no_historical_cancel':
        for obj in [c.master,*c.peers.values()]:obj.cancel_seen.clear()
    for _ in range(40):r=c.tick(mutate=mutate)
    assert not r['ready'] and c.master.receipt is None


@pytest.mark.parametrize('reason',['SHADOW_TOPOLOGY_LOST','IR_APPROACH_CONTRACT_INVALID'])
def test_downstream_task_failure_retires_with_real_cancel_history_and_current_STOP(reason):
    c=interrupted();c.tick()
    def mutate(d):d['guard'].update(state='FAILED',blockers=[reason])
    for _ in range(60):
        r=c.tick(mutate=mutate)
        if r['ready']:break
    assert r['ready'] and set(c.resets.values())=={1}
    assert all(peer.cancel_seen for peer in c.peers.values())
