"""Explicit supervised TOO_CLOSE test; retain raw band and all other gates."""
from fire_robot_mission.pump_demo_core import PumpDemoCore
from fire_robot_mission.ir_approach_shadow_core import validated_ir_band

POLICY='CLOSE_STATIONARY_TEST_V1'


def respray_context(session,now):
    """Actual previous pulse home and same released cycle; no I/motion grant."""
    from .no_goal_cycle import same_cycle
    from .cycle_release import ReleaseContext
    baseline=getattr(session,'respray_after_home',None)
    try:context=ReleaseContext.parse(getattr(session,'r5_cycle_context',None)).as_dict()
    except (ValueError,TypeError):return False
    a=session.actuator(now)
    own_pulse=(session.used and session.phase in ('ARMING','FIRING')
        and session.pump.state=='FIRING' and session.pump.on_since is not None
        and session.pump.r5_cycle_context==context)
    return bool(type(baseline) is int and baseline>=0 and a and (session.idle(a) or own_pulse)
        and a['home_command_completed_count']>baseline
        and all(same_cycle(session.get(k,now),context) for k in ('alignment','guard','gateway','ir_live')))


def close_ir_stop_context(session, now):
    """R5 only: a real forward close-range stop, with the raw band retained."""
    live=session.get('ir_live',now) or {}
    context=getattr(session,'r5_cycle_context',None)
    return bool(session.allow_close_stationary and getattr(session,'unbounded_approach',False)
        and isinstance(context,dict) and live.get('r5_cycle_context')==context
        and live.get('unbounded_approach') is True and live.get('state')=='COMPLETE'
        and live.get('stop_reason')=='CLOSE_RANGE_REACHED' and live.get('publisher_owned') is False
        and live.get('command')=={'linear_x_m_s':0.,'angular_z_rad_s':0.}
        and live.get('recovery_required') is False)


def allowed_bands(session, now):
    request=session.get('stationary_request',now) or {}
    close=(session.allow_close_stationary and
        (request.get('range_policy')==POLICY or respray_context(session,now) or close_ir_stop_context(session,now)))
    return ('SPRAY_RANGE','TOO_CLOSE') if close else ('SPRAY_RANGE',)


class StationaryTestPumpCore(PumpDemoCore):
    def __init__(self, session, **kwargs):
        super().__init__(**kwargs)
        self.session=session

    def blockers(self, now, counts):
        reasons=super().blockers(now,counts)
        s=self.session;ir=self._input('ir',now);request=s.get('stationary_request',now) or {}
        confirmation=s.stationary
        # Only the actor's independently verified explicit-I context can admit
        # this test band. A bare TOO_CLOSE message or early P cannot enable ON.
        close_context=(s.allow_close_stationary and request.get('range_policy')==POLICY
            and request.get('active') is True and confirmation.state in ('CONFIRMING','COMPLETE')
            and request.get('request_id')==confirmation.token and request.get('goal_uuid')==confirmation.goal
            and confirmation.band=='TOO_CLOSE')
        close_context=close_context or s.allow_close_stationary and respray_context(s,now) or close_ir_stop_context(s,now)
        if (close_context and isinstance(ir,dict) and type(ir.get('schema_version')) is int and ir['schema_version']==1
                and validated_ir_band(ir)=='TOO_CLOSE' and ir.get('zone')=='TOO_CLOSE' and ir.get('blockers')==[]
                and ir.get('mode')=='OBSERVE_ONLY' and ir.get('observe_only') is True
                and ir.get('calibration')=='PROVISIONAL_SINGLE_FLAME' and ir.get('validated_for_actuation') is False
                and (ir.get('safety') or {}).get('motion_command_publisher') is False
                and (ir.get('safety') or {}).get('actuator_command_publisher') is False):
            # All sensor age/sequence/stability conditions are additionally
            # enforced by common_errors at confirmation, registration and ON.
            reasons=[r for r in reasons if r!='IR_NOT_CONFIRMED_SPRAY_RANGE']
        return reasons
