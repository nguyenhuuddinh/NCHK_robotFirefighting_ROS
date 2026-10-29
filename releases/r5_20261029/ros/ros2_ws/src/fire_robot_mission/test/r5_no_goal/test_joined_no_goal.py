"""Single no-goal P timeline: real operator/actor/review with fake DDS/device.

No goal result/cancel is fabricated. Gateway/guard witness cores execute;
other peer release state machines use fake I/O. This is not hardware proof.
"""
import copy
import json
from rclpy.qos import ReliabilityPolicy as RP
from std_msgs.msg import String
from fixtures import StopPair
from test_ir_pulse import no_goal_data
from test_installed_pulse import adapter as actor_fixture,endpoint
from test_operator_live_delta import operator_fixture
from test_release_core import Chain
from fire_robot_mission.operational_review import OperationalReview
from fire_robot_mission.operational_intents import ARM_TOPICS
from fire_robot_mission.operational_identity import IDENTITY_TOPIC
from fire_robot_mission.cycle_release import CONTROL_TOPIC
from fire_robot_mission.operational_contract import OPERATOR_NODE,PULSE_STATE_TOPIC


def test_no_goal_stationary_I_P_home_review_ACK_new_goal_same_operator(monkeypatch):
    actor=actor_fixture(monkeypatch,cycle_protocol=True);review=OperationalReview();pair=StopPair()
    box={};pulse_at=[None];home_at=[None]
    actor.pubs[CONTROL_TOPIC]=[endpoint(OPERATOR_NODE,'std_msgs/msg/String',RP.RELIABLE,90)]
    actor.pubs[IDENTITY_TOPIC]=[endpoint('serial_bridge_node',seed=91)]
    for t,name in [('/fire_mission/state','fire_mission_supervisor'),('/fire_mission/live_nav_stop_state','live_nav_stop_gateway'),
                   ('/fire_mission/live_alignment_state','live_alignment_takeover'),('/fire_mission/ir_approach_live_state','ir_approach_live')]:
        actor.pubs[t]=[endpoint(name)]
    def destroy(pub):actor.pubs.pop('/pump_cmd',None)
    monkeypatch.setattr(actor.node,'destroy_publisher',destroy)
    def facts(now,d):
        pair.now=now-.1;pair.tick()
        d['gateway']=copy.deepcopy(pair.g);d['guard']=copy.deepcopy(pair.c)
        d['mission']['navigation'].update(goal_state='IDLE_OR_UNKNOWN',goal_active=False)
        d['alignment'].update(goal_uuid=None,cycle_id=pair.context['cycle_id'],r5_cycle_context=pair.context,
            state='COMPLETE',stop_reason='TARGET_CENTERED',blockers=[],command=dict(linear_x_m_s=0.,angular_z_rad_s=0.))
        d['ir_live'].update(goal_uuid=None,cycle_id=None,r5_cycle_context=pair.context,
            state='WAITING_FOR_ADMISSION',one_shot=True,step_count=0,total_progress_m=0.,stop_reason='',
            publisher_owned=False,recovery_required=False,blockers=['OPERATOR_ARM_MISSING_OR_STALE'],
            safety=dict(pump_command_publisher=False))
        for role in ('mission','gateway','guard','alignment','ir_live','actor'):d.setdefault(role,{})['no_goal_supported']=True
    def actor_input(now,d):
        op=box['op'];a=d['actuator'];previous=actor.sent.get('/pump_cmd',[])
        if previous and previous[-1]:
            if pulse_at[0] is None:pulse_at[0]=now
            a.update(pump_pwm_commanded_on=True,servo_pwm_commanded_on=True,servo_phase='SWEEPING',servo_commanded_angle_deg=60.)
        elif pulse_at[0] is not None:
            if home_at[0] is None:home_at[0]=now+1.2
            a.update(pump_pwm_commanded_on=False,servo_phase='RETURNING_HOME')
            if now>=home_at[0]:
                a.update(servo_pwm_commanded_on=False,servo_phase='IDLE',servo_commanded_angle_deg=None,home_command_completed_count=1)
                d['vision']['fire']=dict(detected_now=False,confirmed=False)
                d['ir']=copy.deepcopy(op.clear_ir);d['ir']['ir_sample_seq']+=round(now*10)
        # Produce witnesses before feeding the actor. fact_source runs after
        # the callback as well, so every participant consumes the same cycle.
        facts(now,d)
        control=op.sent[CONTROL_TOPIC][-1] if op.sent[CONTROL_TOPIC] else None
        if control:actor.node._cycle.on_control(String(data=json.dumps(control)))
        for key in ('identity','actuator','guard','status'):actor.node._cycle.participant.update(key,d[key],now)
        actor.node._cycle.participant.update('cmd',[0.]*6,now);actor.node._cycle.participant.update('odom',[0.]*6,now)
        stationary=op.sent.get('/fire_mission/r4d_stationary_request',[])
        if stationary:actor.node.transport.json_input('stationary_request',json.dumps(stationary[-1]),now)
        arm=op.sent[ARM_TOPICS[-1]][-1] if op.sent[ARM_TOPICS[-1]] else False
        actor.feed(d,now,arm=arm)
        # Fixture feed has an independent inactive stationary default; replace
        # it with the actual operator request before running the actor tick.
        if stationary:actor.node.transport.json_input('stationary_request',json.dumps(stationary[-1]),now)
        # Fixture status uses a legacy canceled goal; replace with actual empty
        # stream. It is never permitted into the production core.
        actor.node.session.update('status',[],now)
        actor.node._cycle.participant.update('status',[],now)
        actor.node.tick();op.pubs['/pump_cmd']=copy.deepcopy(actor.pubs.get('/pump_cmd',[]))
        return json.loads(actor.sent[PULSE_STATE_TOPIC][-1])
    def review_input(now,d):
        for key,source in [('actor','actor'),('vision','vision'),('ir','ir'),('alignment','preview'),('mission','mission')]:review.update(key,d[source],now)
        return review.step(now,graph_ok=True)
    op=operator_fixture(monkeypatch,actor_source=actor_input,review_source=review_input,fact_source=facts);box['op']=op
    op.node.core.cycles._id_factory=lambda:pair.context['cycle_id'];op.d['status']=[]
    f=Chain(no_goal=True)
    for _ in range(6):f.tick()
    op.clear_ir=copy.deepcopy(f.master.inputs['ir'][0])
    for _ in range(35):op.tick()
    assert op.tick('A')['armed']
    op.d['vision']['fire']=dict(detected_now=True,confirmed=True)
    r=op.tick('F');assert r['cycle']['nav_goal_uuid'] is None
    for _ in range(10):
        r=op.tick()
        if r['phase']=='WAIT_IR_I':break
    assert r['phase']=='WAIT_IR_I',r
    assert op.tick('I')['operator_feedback']['accepted']
    for _ in range(10):
        op.d['ir']['ir_sample_seq']+=1;r=op.tick()
        if r['phase']=='WAIT_P' and actor.node.session.stationary.state=='COMPLETE':break
    assert r['phase']=='WAIT_P' and actor.node.session.stationary.state=='COMPLETE',(r,actor.node.session.stationary.report())
    assert op.tick('P')['operator_feedback']['accepted']
    for _ in range(160):
        r=op.tick()
        if r['handback']['consumed']:break
    assert r['handback']['consumed'],(r,actor.node.session.report(False,[]))
    assert r['cycle'] is None and not r['closed'] and r['reason']=='NAV2_READY_NEW_GOAL'
    assert pulse_at[0] is not None and actor.node.session.phase=='WAIT_P'
    assert pair.guard.sent_at is None and pair.guard.send_count==0 and pair.guard.goal_uuid is None
    assert all(value['context']['nav_goal_uuid'] is None for value in op.sent[CONTROL_TOPIC])
    op.d['status']=[('56'*16,2)];r=op.tick()
    assert r['phase']=='NAVIGATION' and op.sent[ARM_TOPICS[0]][-1]
