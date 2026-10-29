"""Pure input/endpoint adapter; no ROS imports and no I/O."""
import json
import math
from .pulse_session import number,uuid_ok
from .pump_demo_core import zero_twist


from .operational_contract import exact_endpoint as exact


def six(twist):
    values=(twist.linear.x,twist.linear.y,twist.linear.z,
            twist.angular.x,twist.angular.y,twist.angular.z)
    return values if all(number(v) for v in values) else None


class Transport:
    def __init__(self,session):
        self.session=session
        self.aged={}
        self.arm_last=None
        self.arm_at=None
        self.pending_p=None
        self.request_index = 0
        self.request_feedback = dict(index=0, accepted=None, reason='NO_P_REQUEST')

    def cancel_unused(self,baseline,now):
        """Acknowledge an authenticated revoked edge which never entered ON."""
        s=self.session
        if (type(baseline) is not int or baseline<0 or self.request_index not in (baseline,baseline+1)
                or s.used or s.phase!='WAIT_P' or s.pump.on_since is not None
                or not s.idle(s.actuator(now))
                or not zero_twist(s.get('cmd',now),.001)
                or not zero_twist(s.get('odom',now),.02)):
            return False
        self.request_index=baseline+1;self.pending_p=None
        self.arm_last=False;self.arm_at=now;s.update('operator',False,now)
        self.request_feedback=dict(index=self.request_index,accepted=False,
            reason='P_CANCELLED_BEFORE_ADMISSION')
        return True

    def json_input(self,name,text,now):
        try:
            payload=json.loads(text)
            if not isinstance(payload,dict):payload=None
        except (ValueError,TypeError):payload=None
        self.session.update(name,payload,now)
        if name in ('vision','mission','preview'):
            age=None
            if isinstance(payload,dict):
                nested=payload.get('perception') if name=='mission' else payload.get('source')
                if isinstance(nested,dict):age=nested.get('source_age_ms' if name=='mission' else 'age_ms')
            self.aged[name]=(age,now)

    def arm(self,value,now):
        if value is True and self.arm_last is False and self.arm_at is not None and 0<=now-self.arm_at<=.4:
            self.request_index += 1
            self.pending_p=now
        self.arm_last=value is True;self.arm_at=now
        self.session.update('operator',value is True,now)

    def odom(self,msg,now,clock_ns):
        q=msg.pose.pose.orientation
        age=(clock_ns-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        values=six(msg.twist.twist)
        if (msg.header.frame_id!='odom' or msg.child_frame_id!='base_link'
                or not all(number(v) for v in (q.x,q.y,q.z,q.w))
                or not .9<=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<=1.1
                or not -.05<=age<=.3):values=None
        self.session.update('odom',values,now);self.aged['odom']=(age*1000,now)

    def status(self,msg,now):
        rows=[]
        try:
            for item in msg.status_list:
                uid=bytes(item.goal_info.goal_id.uuid).hex()
                if not uuid_ok(uid) or type(item.status) is not int or not 0<=item.status<=6:raise ValueError()
                rows.append((uid,item.status,item.goal_info.stamp.sec,item.goal_info.stamp.nanosec))
        except (ValueError,TypeError,AttributeError):rows=[]
        self.session.update('status',rows,now)

    def fresh_counts(self,now,counts):
        """Gate direct image evidence too; fresh mission summaries cannot replace it."""
        counts=dict(counts)
        committed=self.session.committed()
        for name,(age,received) in self.aged.items():
            if committed and name in ('vision','preview','mission'):continue
            lower=-.05 if name=='odom' else 0.
            if not number(age) or not lower<=(age/1000+now-received)<=.3:
                self.session.update(name,None,received)
        for name in ('vision','preview'):
            if not committed and self.session.get(name,now,.3) is None:
                counts[name]=0
        if not committed and (counts.get('vision')!=1 or counts.get('preview')!=1):
            self.session.update('mission',None,now)
        return counts

    def step(self,now,counts,*,stop=False):
        counts=self.fresh_counts(now,counts)
        attempted = self.pending_p is not None
        request=self.pending_p is not None and 0<=now-self.pending_p<=.1 + 1e-9
        self.pending_p=None
        was_used = self.session.used
        before_round=getattr(self.session,'index',None)
        report = self.session.step(now,counts,request=request,stop=stop)
        if attempted:
            accepted = request and ((not was_used and self.session.used)
                or type(before_round) is int and self.session.index==before_round+1)
            self.request_feedback = dict(index=self.request_index, accepted=accepted,
                reason=('P_ACCEPTED_BY_ACTOR' if accepted else 'P_EDGE_EXPIRED' if not request else
                        report.get('reason') or next(iter(report.get('blockers', [])), 'P_NOT_ADMITTED')))
        report.update(pulse_request_feedback=dict(self.request_feedback),
                      pulse_arm_active=self.arm_last is True,
                      pulse_request_pending=self.pending_p is not None)
        return report
