"""Use historical fixture data only. No historical QA is collected or run."""
import copy
import importlib.util
from pathlib import Path
from fire_robot_mission.live_nav_stop_gateway import LiveNavStopCore
from fire_robot_mission.nav2_cancel_guard import Nav2CancelGuardCore
from test_release_core import Chain

def fixture_module(filename):
    path=Path(__file__).parents[1]/filename
    spec=importlib.util.spec_from_file_location('fixture_'+path.stem,path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

class StopPair:
    def __init__(self):
        self.context=Chain(no_goal=True).context;self.now=1.
        self.gw=LiveNavStopCore(enabled=True);self.guard=Nav2CancelGuardCore(enabled=True)
        self.gw.r5_cycle_context=copy.deepcopy(self.context);self.guard.r5_cycle_context=copy.deepcopy(self.context)
        self.nav=fixture_module('test_live_nav_stop_gateway.py')
        self.g={};self.c={}
    def tick(self,*,active=None,raw=(),moving=False,graph=True):
        self.now+=.1;n=self.now;z=(0.,)*6;rows=[] if active is None else [(active,2)]
        self.gw.update_statuses(rows,n);self.guard.update_statuses(rows,n)
        self.gw.update_mission(self.nav.mission('FIRE_TRACKING','HOLD_STOP',goal=False),n)
        for core in (self.gw,self.guard):
            core.update_odom((0.,0,0,0,0,.15) if moving else z,n);core.update_cmd(z,n);core.update_arm(True,n)
        self.gw.update_nav(z,n)
        _,_,self.g,action=self.gw.evaluate(n,False,list(raw),self.nav.NAV_SOURCES if graph else [],self.nav.SAFETY_SOURCE,1,1,1,1,1)
        self.guard.update_topology(1,1,1,self.nav.NAV_SOURCES if graph else [],len(raw),1,1,False,raw_sources=raw,ir_state_source_ok=True)
        self.c,sent=self.guard.step(n)
        assert sent is None
        return self.g,self.c
    def qualify(self):
        for _ in range(8):self.tick()
        assert self.g['no_goal_proof']['stationary_released'] and self.c['no_goal_proof']['stationary_released']
        return self.g,self.c
