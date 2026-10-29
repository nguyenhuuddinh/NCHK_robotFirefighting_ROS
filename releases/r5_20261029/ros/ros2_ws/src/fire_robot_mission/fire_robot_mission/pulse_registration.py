"""Register the pump endpoint only after fresh complete IR stop; no commands."""


def registration_ready(session,now,counts,total_pump_publishers):
    if hasattr(session,'registration_session'):
        session=session.registration_session()
        if session is None:return False
    if (session.phase!='WAIT_P' or session.used or not session.pump.enabled
            or total_pump_publishers!=0 or counts.get('raw')!=0
            or counts.get('vision')!=1 or counts.get('preview')!=1):return False
    # Feed the actual baseline permission checker. Before endpoint registration
    # its own publisher is necessarily absent and operator P is still false.
    # These two expected preconditions may be omitted only for *registration*;
    # real ON admission continues to require both, without exception.
    session.feed_pump(now)
    blockers=session.pump.blockers(now,counts)+session.extra_blockers(now,counts,require_idle=True)
    return not set(blockers)-{'PUMP_OWNER_CONFLICT','OPERATOR_ARM_MISSING_OR_STALE'}
