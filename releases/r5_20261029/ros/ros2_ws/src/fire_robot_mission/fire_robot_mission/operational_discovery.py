"""Zero-only raw name discovery for initial IR and R5 Nav2; no command output.

Nav2 also supplies an explicit owner; IR retains its existing context policy.
"""
import copy

NAMES={'live_nav_stop_gateway','live_alignment_takeover','ir_approach_live'}
UNKNOWN=('_NODE_NAME_UNKNOWN_','_NODE_NAMESPACE_UNKNOWN_')
LIMIT_S=.75

class Discovery:
    def __init__(self):
        self.since = None
        self.gid = self.goal = self.cycle = None
        self.resolved = False
        self.failed = False

    def observe(self, now, endpoints, context):
        status, reason = 'INVALID', 'RAW_ENDPOINT_INVALID'
        expected=context.get('expected_owner') or 'ir_approach_live'
        if self.failed:
            reason = 'RAW_DISCOVERY_ALREADY_FAILED'
        elif self.since is not None and not self.resolved and now-self.since >= LIMIT_S:
            reason = 'RAW_DISCOVERY_DEADLINE'
        elif not endpoints:
            status, reason = 'EMPTY', ''
        elif len(endpoints) == 1:
            p = endpoints[0]
            try:
                gid_bytes = bytes.fromhex(p.get('gid', ''))
                gid_ok = len(gid_bytes) >= 16 and any(gid_bytes)
            except (TypeError, ValueError):
                gid_ok = False
            metadata = (gid_ok and p.get('type') == 'geometry_msgs/msg/Twist'
                        and p.get('reliability') == 'ReliabilityPolicy.BEST_EFFORT')
            exact = p.get('node') in NAMES and p.get('namespace') == '/'
            unknown = (p.get('node'), p.get('namespace')) == UNKNOWN
            same = self.gid is None or (p.get('gid') == self.gid and
                                       context.get('goal_uuid') == self.goal and context.get('cycle_id')==self.cycle)
            if metadata and exact and (not context.get('expected_owner') or p['node']==expected):
                if self.since is not None and not self.resolved and (not same or p['node'] != expected):
                    reason = 'RAW_DISCOVERY_IDENTITY_CHANGED'
                else:
                    self.resolved = self.resolved or self.since is not None
                    status, reason = 'EXACT', ''
            elif metadata and unknown and context.get('valid') is True and same and not self.resolved:
                if self.since is None:
                    self.since, self.gid, self.goal = now, p['gid'], context.get('goal_uuid')
                    self.cycle=context.get('cycle_id')
                status, reason = 'PENDING', ('INITIAL_NAV_NAME_DISCOVERY_ZERO_ONLY'
                    if expected=='live_nav_stop_gateway' else 'INITIAL_IR_NAME_DISCOVERY_ZERO_ONLY')
            elif unknown:
                reason = 'RAW_UNKNOWN_OUTSIDE_ZERO_ADMISSION'
        if status == 'INVALID':
            self.failed = True
        return dict(status=status, reason=reason, observed_at=now, since=self.since,
                    limit_s=LIMIT_S, goal_uuid=self.goal, cycle_id=self.cycle, endpoints=copy.deepcopy(endpoints),
                    context=copy.deepcopy(context))


