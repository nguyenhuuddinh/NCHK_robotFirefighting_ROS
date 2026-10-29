"""One timeline: installed pulse adapter -> canonical review -> reset -> new goal.

Operator/actor/adapters/cores are real, DDS/device I/O fake; five other peers
execute the release state machine. This is not a six-ROS-node/hardware test.
"""
import copy
import json
from types import SimpleNamespace as N
from rclpy.qos import ReliabilityPolicy as RP
from std_msgs.msg import String
from fire_robot_mission.operational_review import OperationalReview
from fire_robot_mission.operational_intents import ARM_TOPICS
from fire_robot_mission.operational_identity import IDENTITY_TOPIC
from fire_robot_mission.cycle_release import CONTROL_TOPIC
from fire_robot_mission.operational_contract import PULSE_STATE_TOPIC,OPERATOR_NODE
from test_installed_pulse import adapter as actor_fixture,endpoint
from test_operator_live_delta import operator_fixture


def test_joined_real_actor_P_OFF_home_review_release_and_operator_new_goal(monkeypatch):
    actor=actor_fixture(monkeypatch,cycle_protocol=True);review=OperationalReview();box={};pulse_at=[None];home_at=[None]
    actor.pubs[CONTROL_TOPIC]=[endpoint(OPERATOR_NODE,'std_msgs/msg/String',RP.RELIABLE,90)]
    actor.pubs[IDENTITY_TOPIC]=[endpoint('serial_bridge_node',seed=91)]
    def destroy(pub):actor.pubs.pop('/pump_cmd',None)
    monkeypatch.setattr(actor.node,'destroy_publisher',destroy)
    def actor_input(now,d):
        operator=box['operator'];a=d['actuator'];previous=actor.sent.get('/pump_cmd',[])
        # Both the pulse transport and release participant receive the same
        # advancing MCU report; adapter.feed alone only updates its own copy.
        a['esp_ms']=round(now*1000)
        on=bool(previous and previous[-1])
        if on:
            if pulse_at[0] is None:pulse_at[0]=now
            a.update(pump_pwm_commanded_on=True,servo_pwm_commanded_on=True,servo_phase='SWEEPING',servo_commanded_angle_deg=60.)
        elif pulse_at[0] is not None:
            if home_at[0] is None:home_at[0]=now+1.2
            a.update(pump_pwm_commanded_on=False,servo_phase='RETURNING_HOME')
            if now>=home_at[0]:
                a.update(servo_pwm_commanded_on=False,servo_phase='IDLE',servo_commanded_angle_deg=None,home_command_completed_count=1)
                d['vision']['fire']=dict(detected_now=False,confirmed=False)
                d['ir']=copy.deepcopy(operator.clear_ir);d['ir']['ir_sample_seq']+=round(now*10)
        control=operator.sent[CONTROL_TOPIC][-1] if operator.sent[CONTROL_TOPIC] else None
        if control:
            actor.node._cycle.on_control(String(data=json.dumps(control)))
        for name,value in d.items():
            if name in ('identity','actuator','guard','status'):actor.node._cycle.participant.update(name,value,now)
        actor.node._cycle.participant.update('cmd',[0.]*6,now)
        actor.node._cycle.participant.update('odom',[0.]*6,now)
        arm=operator.sent[ARM_TOPICS[-1]][-1] if operator.sent[ARM_TOPICS[-1]] else False
        actor.feed(d,now,arm=arm);actor.node.tick()
        operator.pubs['/pump_cmd']=copy.deepcopy(actor.pubs.get('/pump_cmd',[]))
        return json.loads(actor.sent[PULSE_STATE_TOPIC][-1])
    def review_input(now,d):
        for name,key in [('actor','actor'),('vision','vision'),('ir','ir'),('alignment','preview'),('mission','mission')]:
            review.update(name,d[key],now)
        return review.step(now,graph_ok=True)
    op=operator_fixture(monkeypatch,actor_source=actor_input,review_source=review_input);box['operator']=op
    from test_release_core import Chain
    facts=Chain()
    for _ in range(6):facts.tick()
    op.clear_ir=copy.deepcopy(facts.master.inputs['ir'][0])
    for _ in range(35):op.tick()
    assert op.tick('A')['armed']
    op.d['vision']['fire']=dict(detected_now=True,confirmed=True);op.tick('F')
    op.d['status']=[('a'*32,5)];op.tick();op.tick('T')
    op.d['alignment'].update(state='COMPLETE',goal_uuid='a'*32,stop_reason='TARGET_CENTERED',blockers=[])
    # Begin explicit I from APPROACH with idle live node, then supply real R4
    # movement-complete contract. No fake pump admission replaces explicit P.
    op.d['ir'].update(ir_band='APPROACH',zone='APPROACH',filtered_counts=dict(left=2000,center=3000,right=5000))
    op.d['ir_live'].update(state='WAITING_FOR_ADMISSION',goal_uuid=None,step_count=0,total_progress_m=0.,stop_reason='',publisher_owned=False)
    assert op.tick()['phase']=='WAIT_IR_I'
    r=op.tick('I');assert r['operator_feedback']['accepted']
    op.d['ir_live'].update(state='MOVING',goal_uuid='a'*32,step_count=0,total_progress_m=.02,publisher_owned=True)
    op.tick()
    op.d['ir_live'].update(state='COMPLETE',goal_uuid='a'*32,step_count=1,total_progress_m=.04,
                          stop_reason='SPRAY_RANGE_REACHED',publisher_owned=False)
    op.d['ir'].update(ir_band='SPRAY_RANGE',zone='SPRAY_RANGE',filtered_counts=dict(left=1400,center=5500,right=950))
    for _ in range(20):
        op.d['ir']['ir_sample_seq']+=1;r=op.tick()
        if r['phase']=='WAIT_P':break
    assert r['phase']=='WAIT_P',r
    assert not any(actor.sent.get('/pump_cmd',[]))
    r=op.tick('P');assert r['operator_feedback']['accepted']
    for _ in range(160):
        r=op.tick()
        if r['handback']['consumed']:break
    assert r['handback']['consumed'],(r,actor.node.session.report(False,[]))
    assert pulse_at[0] is not None and home_at[0] is not None
    assert any(actor.sent['/pump_cmd']) and actor.sent['/pump_cmd'][-1] is False
    assert actor.node._cycle.participant.phase=='COMMITTED'
    assert actor.node.session.phase=='WAIT_P' and not actor.node.session.used
    assert r['reason']=='NAV2_READY_NEW_GOAL' and not r['closed']
    op.d['status']=[('56'*16,2)];r=op.tick()
    assert r['phase']=='NAVIGATION' and op.sent[ARM_TOPICS[0]][-1]
