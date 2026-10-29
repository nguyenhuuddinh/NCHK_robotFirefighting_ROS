"""Bounded post-disarm verification, independent of trial success; no ROS/I/O."""
import math
from .actuator_freshness import fresh_off


class Drain:
    def __init__(self, now, home_before=None):
        self.start=now;self.home_before=home_before;self.still_since=None
        self.done=False;self.ok=False;self.reason='WAIT_FRESH_ACTUATOR_OFF'

    def step(self,now,act,received,cmd,cmd_at,odom,odom_at,raw_count):
        def zero(v,t,limit):
            return (isinstance(v,(tuple,list)) and len(v)==6 and t is not None
                and 0<=now-t<=.35 and all(type(x) in (int,float) and math.isfinite(x) and abs(x)<=limit for x in v))
        home=bool(self.home_before is None or (isinstance(act,dict)
            and type(act.get('home_command_completed_count')) is int
            and act['home_command_completed_count']>self.home_before))
        safe=(fresh_off(act,received,now) and home and raw_count==0
              and zero(cmd,cmd_at,.001) and zero(odom,odom_at,.02))
        self.still_since=(now if self.still_since is None else self.still_since) if safe else None
        if self.still_since is not None and now-self.still_since>=.4:
            self.done=self.ok=True;self.reason='FRESH_OFF_HOME_ZERO_RELEASE_CONFIRMED'
        elif now-self.start>=4:
            self.done=True;self.reason='ACTUATOR_OR_MOTION_STOP_UNCONFIRMED'
        return {'done':self.done,'confirmed':self.ok,'reason':self.reason,'elapsed_s':now-self.start}
