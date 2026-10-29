import pytest
from harness import Robot
from test_full_chain import align
from fire_robot_mission.operational_intents import ARM_TOPICS


@pytest.mark.parametrize('phase',['standby','nav','turn','IR','WAIT_P','pulse','review'])
def test_Q_keeps_actual_six_alive_until_zero_OFF_home_release(monkeypatch,phase):
    h=Robot(monkeypatch,goal=phase=='nav');h.arm()
    if phase=='nav':h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    elif phase in ('turn','IR','WAIT_P','pulse','review'):
        h.fire=True;h.step('F');h.until(lambda r:r['phase']=='WAIT_T');h.step('T')
        h.until(lambda r:abs(h.yaw)>.025)
        if phase!='turn':
            h.centered=True;h.until(lambda r:r['phase']=='WAIT_IR_I');h.step('I')
            h.until(lambda r:h.x>.025)
            if phase!='IR':
                h.right=950;h.until(lambda r:r['phase']=='WAIT_P')
                if phase in ('pulse','review'):
                    h.step('P');h.until(lambda r:h.on_at is not None)
                    if phase=='review':h.until(lambda r:h.home==1)
    r=h.step('Q');stop_at=h.bus.now;assert r['closed'] and not r['shutdown']['done']
    # Closing requests false arms, never a new publisher or queued P.
    controls=sum(t=='/fire_mission/cycle_control' for _,_,t,_ in h.bus.sent)
    r=h.until(lambda r:r.get('shutdown',{}).get('done'),limit=100)
    assert r['shutdown']['confirmed'],h.diagnostic(r)
    assert not any(p.topic in ('/cmd_vel_raw','/pump_cmd') for p in h.bus.pubs)
    assert controls==sum(t=='/fire_mission/cycle_control' for _,_,t,_ in h.bus.sent)
    assert h.op.intent_executor.closed and not h.op.stop_verification.report(h.bus.now)['motion_authorized']
    assert all(msg.data is False for at,node,topic,msg in h.bus.sent if topic in ARM_TOPICS and at>=stop_at)
    if phase in ('pulse','review'):assert h.home==1 and r['shutdown']['home_required']


def test_early_P_key_without_any_ON_does_not_invent_a_required_home(monkeypatch):
    h=Robot(monkeypatch);h.arm();h.step('P');h.step('Q')
    r=h.until(lambda r:r.get('shutdown',{}).get('done'))
    assert r['shutdown']['confirmed'] and not r['shutdown']['home_required'] and h.on_at is None


def test_Q_real_monotonic_advances_inside_tick_does_not_false_clock_fault(monkeypatch):
    from fire_robot_mission import operational_session as ros
    h=Robot(monkeypatch);h.arm();calls=[0]
    def clock():calls[0]+=1;return h.bus.now+calls[0]*.000001
    monkeypatch.setattr(ros.time,'monotonic',clock)
    r=h.op.tick('Q')
    assert r['shutdown']['fault'] is None and not r['shutdown']['done']
    assert r['shutdown']['reason']=='WAIT_STOP_PROOF'
