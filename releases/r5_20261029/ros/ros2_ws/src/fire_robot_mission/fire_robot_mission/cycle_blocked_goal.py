"""Cancel a foreign RViz goal during a fire cycle or retirement, never drive.

Use the existing cancel guard client. Preserve the original cycle/cancel
receipt and require the new goal's actual terminal status before unblocking.
"""
from .operational_cycle import NavigationSnapshot
from .nav2_cancel_guard import make_cancel_request


class BlockedGoalCancel:
    def __init__(self):
        self.context=None;self.requests={};self.last=None

    def step(self,endpoint,now):
        p=endpoint.participant;node=endpoint.node;ctx=p.context
        if ctx is None:return None
        if self.context!=ctx:
            self.context=ctx;self.requests={};self.last=None
        try:
            rows,at=p.inputs.get('status',(None,None));status=NavigationSnapshot.parse(rows,at)
            if at>now:raise ValueError()
        except (ValueError,TypeError):return dict(state='WAIT_VALID_NAV_STATUS')
        uid=status.active_uuid
        if uid==ctx.nav_goal_uuid:uid=None
        pending=[k for k,v in self.requests.items() if not v.get('terminal')]
        if uid is None and not pending:return None
        if (ctx.nav_goal_uuid is not None and
                [s for u,s in status.rows if u==ctx.nav_goal_uuid] not in ([],[5])):
            return dict(state='WAIT_ORIGINAL_CANCEL_STATUS')
        # Only STOP permits a request. It never grants motion or retires peers.
        errors=p.stop_errors(ctx,now,endpoint.graph_ok,
            len(node.get_publishers_info_by_topic('/cmd_vel_raw')),
            len(node.get_publishers_info_by_topic('/pump_cmd')))
        independent=set(errors)-{'WAIT_NO_ACTIVE_GOAL','WAIT_REAL_GOAL_CANCELED'}
        history=(ctx.nav_goal_uuid is None or
            (ctx.cycle_id,ctx.nav_goal_uuid,ctx.identity) in p.cancel_seen)
        if independent or not history:
            return dict(state='WAIT_BLOCKED_GOAL_STOP',blockers=sorted(independent),original_cancel_seen=history)
        if uid is not None and uid not in self.requests:
            if not node.cancel_client.service_is_ready():return dict(state='WAIT_CANCEL_SERVICE',goal_uuid=uid)
            try:
                self.requests[uid]=dict(future=None,since=now,accepted=False,terminal=None)
                future=node.cancel_client.call_async(make_cancel_request(uid))
                self.requests[uid]['future']=future
            except Exception:
                return dict(state='BLOCKED_GOAL_CANCEL_SEND_FAILED',goal_uuid=uid)
        for target,r in self.requests.items():
            if r.get('terminal'):continue
            f=r['future']
            if f is None:return dict(state='BLOCKED_GOAL_CANCEL_SEND_FAILED',goal_uuid=target)
            if f.done() and not r['accepted']:
                try:
                    response=f.result()
                    ids=[bytes(g.goal_id.uuid).hex() for g in response.goals_canceling]
                    r['accepted']=response.return_code==0 and ids==[target]
                except Exception:r['accepted']=False
            actual=[s for u,s in status.rows if u==target]
            if actual in ([4],[5],[6]):
                r['terminal']=actual[0]
                self.last=dict(state='BLOCKED_GOAL_TERMINAL',goal_uuid=target,
                    cancel_accepted=r['accepted'],actual_terminal=actual[0])
            elif now-r['since']>3.:
                return dict(state='BLOCKED_GOAL_CANCEL_UNCONFIRMED',goal_uuid=target)
            else:return dict(state='WAIT_BLOCKED_GOAL_TERMINAL',goal_uuid=target,cancel_accepted=r['accepted'])
        if status.active_uuid is not None:return dict(state='WAIT_NO_ACTIVE_GOAL')
        # Only the known foreign-goal safety failure can be restored here,
        # after real terminal evidence and all independent current STOP gates.
        if getattr(node.core,'failure',None)=='POST_CANCEL_SAFETY_LOST':
            node.core.failure=None
        return None
