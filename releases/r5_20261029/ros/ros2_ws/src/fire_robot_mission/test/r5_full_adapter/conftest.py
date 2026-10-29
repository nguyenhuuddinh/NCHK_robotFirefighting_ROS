import pytest
import rclpy
import json
from pathlib import Path

@pytest.fixture(autouse=True)
def forbid_runtime(monkeypatch,request):
    from harness import Robot
    Robot.instances=[]
    assert not rclpy.ok()
    monkeypatch.setattr(rclpy,'init',lambda *a,**k:pytest.fail('real ROS context forbidden'))
    yield
    assert not rclpy.ok()
    evidence=Path(__file__).resolve().parents[6]/'evidence_r5_3_full_adapter'
    # Keep every attempt; final per-case QA verdict comes from the XMLs.
    for h in Robot.instances:
        events=[];acks=[];previous=None;last_ack={}
        for at,node,topic,msg in h.bus.sent:
            if not hasattr(msg,'data') or not isinstance(msg.data,str):continue
            value=json.loads(msg.data)
            if topic=='/fire_mission/operator_state':
                key=(value['phase'],value['reason'])
                if key!=previous:
                    events.append(dict(at=at,phase=key[0],reason=key[1],cycle=value['cycle']));previous=key
            ack=value.get('cycle_release_ack')
            if ack is not None:
                key=(ack['phase'],str(ack['blockers']),str(ack['context']))
                if last_ack.get(node)!=key:
                    acks.append(dict(at=at,node=node,ack=ack));last_ack[node]=key
        record=dict(case=request.node.nodeid,scope='IN_MEMORY_REAL_SIX_ADAPTERS_NOT_DDS_OR_HARDWARE',
            operator_events=events,ACK_events=acks,owner_events=h.bus.owner_events,
            cancel_requests=h.bus.cancelled,simulated_pulses=h.pulses,
            simulated_distance_m=h.x,simulated_yaw_rad=h.yaw,max_raw_owners=h.bus.max_raw,
            final_operator=h.reports[-1] if h.reports else None)
        with (evidence/'test_timelines.jsonl').open('a') as f:f.write(json.dumps(record,ensure_ascii=False)+'\n')
