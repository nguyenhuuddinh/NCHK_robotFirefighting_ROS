import copy
import pytest
from fixtures import StopPair,fixture_module
from test_installed_pulse import ready,adapter
from fire_robot_mission.ir_approach_live import IrApproachLiveNode
from fire_robot_mission.pulse_session import PulseSession
from fire_robot_mission.ir_approach_live_core import IrApproachLiveCore
from fire_robot_mission.no_goal_cycle import stopped_pair


def no_goal_data():
    pair=StopPair();g,c=pair.qualify();d=ready();ctx=pair.context
    d.update(gateway=g,guard=c,status=[])
    d['alignment'].update(goal_uuid=None,cycle_id=ctx['cycle_id'],r5_cycle_context=ctx,blockers=[],command=dict(linear_x_m_s=0.,angular_z_rad_s=0.))
    d['ir_live'].update(goal_uuid=None,cycle_id=ctx['cycle_id'],r5_cycle_context=ctx)
    d['mission']['navigation']['goal_state']='IDLE_OR_UNKNOWN'
    return pair,d


class IR:
    def __init__(self):
        self.pair,self.d=no_goal_data();self.m=fixture_module('test_ir_approach_live_core.py')
        self.core=self.m.continuous_core();self.core.r5_cycle_context=self.pair.context
        self.node=object.__new__(IrApproachLiveNode);self.node.core=self.core;self.node._statuses=[]
        self.node._proofs={};self.counts=self.m.integrated_counts(goal_uuid=None)
    def step(self,now,*,owned=False,x=0.,speed=0.,band='APPROACH',fault=None):
        self.m.refresh(self.core,now,x=x,cmd=speed,band=band)
        self.core.update('ir',dict(self.core._input('ir',now),filtered_counts=dict(left=1400,center=5000,right=5000 if band=='APPROACH' else 950)),now)
        self.node._proofs={k:(copy.deepcopy(self.d[k]),now) for k in ('mission','alignment','gateway','guard')}
        self.node._proofs['mission'][0]['perception']['alignment_state']='CENTERED'
        counts=dict(self.counts,raw=1 if owned else 0,raw_owned=owned)
        for k in ('mission','alignment','gateway','guard','status'):counts[k+'_source_ok']=True
        if fault=='cycle':self.node._proofs['alignment'][0]['r5_cycle_context']=dict(self.pair.context,cycle_id='12'*16)
        if fault=='status':self.node._statuses=[('12'*16,2)]
        if fault=='source':counts['guard_source_ok']=False
        if fault=='raw':counts['raw']=2
        if fault=='arm':self.core.update_arm(False,now)
        if fault=='scan':self.core.update('scan',self.m.scan(front=.1),now)
        uid,admission,runtime,range_stop=self.node._handoff_evidence(now,counts)
        counts.update(goal_uuid=uid,handoff_ok=admission,runtime_ok=runtime,range_handoff_ok=range_stop)
        if fault=='forged_goal':counts['goal_uuid']='12'*16
        return self.core.step(now,counts,owned)


def test_real_IR_node_evidence_then_real_core_no_goal_approach_spray_zero_release():
    a=IR();assert a.step(2.)[2] is None
    speed,r,action=a.step(2.31)
    assert action=='CREATE' and r['goal_uuid'] is None and r['cycle_id']==a.pair.context['cycle_id']
    speed,r,_=a.step(2.4,owned=True)
    assert speed==.08 and r['state']=='MOVING'
    speed,r,_=a.step(2.5,owned=True,x=.01,speed=.08,band='SPRAY_RANGE')
    assert speed==0. and r['state']=='STOPPING'
    for i in range(10):
        speed,r,action=a.step(2.6+i*.1,owned=True,x=.01,band='SPRAY_RANGE')
        if action=='RELEASE':break
    assert action=='RELEASE' and r['state']=='COMPLETE' and r['goal_uuid'] is None
    assert r['step_count']==1 and r['total_progress_m']==.01


@pytest.mark.parametrize('fault',['cycle','status','source','raw','arm','scan','forged_goal'])
def test_no_goal_IR_still_requires_context_source_status_and_motion_gates(fault):
    a=IR();a.step(2.,fault=fault);speed,r,action=a.step(2.31,fault=fault)
    assert speed==0. and action is None and r['blockers']


@pytest.mark.parametrize('fault',[None,'wrong_cycle','active_goal','ON','no_IR','raw','wrong_servo'])
def test_real_P_core_no_goal_keeps_same_sensor_ACT_owner_gates(fault):
    pair,d=no_goal_data();s=PulseSession(enabled=True,allow_provisional_ir=True)
    s.r5_cycle_context=pair.context;s.pump.r5_cycle_context=pair.context
    counts={k:1 for k in ('ir','mission','alignment','gateway','guard','status','odom','cmd','arm','pump','pump_subscribers','ir_live','actuator','vision','preview')};counts['raw']=0
    counts['pump_total']=1
    if fault=='wrong_cycle':d['alignment']['r5_cycle_context']=dict(pair.context,cycle_id='12'*16)
    if fault=='active_goal':d['status']=[('12'*16,2,0,0)]
    if fault=='no_IR':d['ir']['blockers']=['IR_TIMEOUT']
    if fault=='ON':d['actuator']['pump_pwm_commanded_on']=True
    if fault=='raw':counts['raw']=1
    def feed(now,operator,request=False):
        for key,value in d.items():
            if key in ('status','odom','cmd') or key in ('ir','mission','alignment','gateway','guard','actuator','vision','preview','ir_live'):s.update(key,copy.deepcopy(value),now)
        s.update('cmd',(0.,)*6,now);s.update('odom',(0.,)*6,now);s.update('operator',operator,now)
        return s.step(now,counts,request=request)
    feed(2.,False);r=feed(2.1,True,True)
    for i in range(7):r=feed(2.2+i*.1,True)
    if fault not in (None,'wrong_servo'):
        assert not r['proposed_pump_on'] and s.phase!='FIRING';return
    assert r['proposed_pump_on'] and r['goal_uuid'] is None and r['cycle_id']==pair.context['cycle_id']
    d['actuator'].update(pump_pwm_commanded_on=True,servo_pwm_commanded_on=True,servo_phase='SWEEPING',servo_commanded_angle_deg=45. if fault=='wrong_servo' else 60.)
    r=feed(2.9,True)
    if fault=='wrong_servo':assert not r['proposed_pump_on'] and r['reason']=='PUMP_SERVO_FEEDBACK_MISMATCH'
    else:assert r['proposed_pump_on'] and r['pump_on_feedback_seen'] and r['pulse_core']['goal_uuid'] is None
