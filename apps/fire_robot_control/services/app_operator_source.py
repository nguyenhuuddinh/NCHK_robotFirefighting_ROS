"""Generate app-only timing fixes from the byte-pinned R5 operator source."""
import importlib.util
from pathlib import Path
import sys


def patch_operator(source):
    from .candidate_build import replace_once
    source = replace_once(source, 'import json\n', 'import json\nfrom services.operator_timing import observe_query\n')
    source = replace_once(source,
        'from .operational_session_core import OperationalSessionCore',
        'from services.app_task_recovery import AppSessionCore as OperationalSessionCore')
    source = replace_once(source,
        'from .operational_handback import OperationalHandback',
        'from services.app_task_recovery import AppHandback as OperationalHandback')
    source = replace_once(source,
        "        if self.stop_verification.start is not None:return self.stop_tick(now)\n",
        "        if self.stop_verification.start is not None:return self.stop_tick(now)\n"
        "        if key=='Q':\n"
        "            self.request_shutdown('Q',now=now)\n"
        "            return self.stop_tick(now)\n")
    source = replace_once(source,
        "        graph=self.graph.check(self,self.core,now,EXPECTED,RP.RELIABLE)\n",
        "        graph=self.graph.check(self,self.core,now,EXPECTED,RP.RELIABLE)\n"
        "        app_raw_count=len(self.get_publishers_info_by_topic('/cmd_vel_raw'))\n"
        "        app_pump_count=len(self.get_publishers_info_by_topic('/pump_cmd'))\n"
        "        checked_at=time.monotonic()\n"
        "        observe_query(self,'graph',checked_at-now,checked_at)\n"
        "        if checked_at-now>.35:\n"
        "            graph=dict(graph,valid=False,app_query_delay_ms=round((checked_at-now)*1000,3))\n"
        "        now=checked_at\n")
    source = replace_once(source,
        "        handback=self.handback.step(self.core,now,graph_ok=self.graph.fault is None\n"
        "            and self.core.get('graph',now).get('valid') is True,\n"
        "            raw_count=len(self.get_publishers_info_by_topic('/cmd_vel_raw')),\n"
        "            pump_count=len(self.get_publishers_info_by_topic('/pump_cmd')))\n",
        "        handback=self.handback.step(self.core,now,graph_ok=self.graph.fault is None\n"
        "            and self.core.get('graph',now).get('valid') is True,\n"
        "            raw_count=app_raw_count,pump_count=app_pump_count)\n")
    source = replace_once(source,
        "        execution=self.intent_executor.step(self.core,report,handback,now)\n",
        "        execute_at=time.monotonic()\n"
        "        observe_query(self,'pre_publish',execute_at-now,execute_at)\n"
        "        if execute_at-now>.35:\n"
        "            self.core.clear_prearm()\n"
        "            self.core.observe_input_hold(execute_at,['OPERATOR_CALLBACK_GAP'])\n"
        "            if self.core.armed:self.core.interrupt_cycle('OPERATOR_CALLBACK_GAP')\n"
        "            self.intent_executor.hold_stopped()\n"
        "            report.update(self.core.report(execute_at))\n"
        "            execution=dict(enabled=self.intent_executor.enabled,live_intents_published=self.intent_executor.enabled,live_intents_active=False,stop_only=True)\n"
        "        else:\n"
        "            report.update(self.core.report(execute_at))\n"
        "            execution=self.intent_executor.step(self.core,report,handback,execute_at)\n"
        "        now=execute_at\n")
    source = replace_once(source,
        "        result=self.stop_verification.step(now,inputs,sources_ok=source_ok and self.stop_source_fault is None,\n"
        "            raw_count=len(self.get_publishers_info_by_topic('/cmd_vel_raw')),\n"
        "            pump_count=len(self.get_publishers_info_by_topic('/pump_cmd')))\n",
        "        raw_count=len(self.get_publishers_info_by_topic('/cmd_vel_raw'))\n"
        "        pump_count=len(self.get_publishers_info_by_topic('/pump_cmd'))\n"
        "        checked_at=time.monotonic()\n"
        "        observe_query(self,'stop',checked_at-now,checked_at)\n"
        "        now=checked_at\n"
        "        result=self.stop_verification.step(now,inputs,sources_ok=source_ok and self.stop_source_fault is None,\n"
        "            raw_count=raw_count,pump_count=pump_count)\n")
    return source


def load_operator_node():
    path = Path(__file__).resolve().parents[1] / 'generated/app_operational_session.py'
    name = 'fire_robot_mission._desktop_operational_session'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return module.OperationalSessionNode
