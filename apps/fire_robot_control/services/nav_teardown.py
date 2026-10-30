"""App-only teardown coordination; never readiness or physical STOP proof."""
import math
import time

GROUPS = {
    'lifecycle_manager_navigation': ('controller_server', 'smoother_server', 'planner_server',
        'behavior_server', 'bt_navigator', 'velocity_smoother'),
    'lifecycle_manager_localization': ('map_server', 'amcl'),
}
NODES = tuple(name for names in GROUPS.values() for name in names)


class SettledPlan:
    """Require stable ACTIVE/finalized groups; never transition an unknown node."""
    HOLD_S = .5
    MAX_GAP_S = .35

    def __init__(self):
        self.since = self.last = self.signature = None

    def observe(self, states, now):
        if not math.isfinite(now):
            self.since = self.last = self.signature = None
            return None
        plan = {}
        if set(states) == set(NODES):
            for manager, names in GROUPS.items():
                values = [states[name] for name in names]
                if all(type(value) is int and value == 3 for value in values):
                    plan[manager] = 'SHUTDOWN'
                elif all(type(value) is int and value == 4 for value in values):
                    plan[manager] = 'FINALIZED'
        signature = tuple(plan.items()) if len(plan) == len(GROUPS) else None
        if self.last is not None and (now < self.last or now-self.last > self.MAX_GAP_S):
            self.since = self.signature = None
        self.last = now
        if signature is None or signature != self.signature:
            self.since, self.signature = (now if signature else None), signature
        return plan if signature and now-self.since >= self.HOLD_S else None


def drain_lifecycle(node, ros, deadline, diagnostic, *, clock=time.monotonic):
    """Wait within the existing RPC budget, then verify actual final node states.

    This runs only AFTER operator/mission STOP in a separately bounded helper.
    Manager responses alone do not imply that container cleanup is clean.
    """
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.srv import ManageLifecycleNodes
    clients = {name: node.create_client(GetState, '/'+name+'/get_state') for name in NODES}
    managers = {name: node.create_client(ManageLifecycleNodes, '/'+name+'/manage_nodes') for name in GROUPS}

    def spin_until(until):
        while clock() < until:
            ros.spin_once(node, timeout_sec=max(0., min(.02, until-clock())))

    def snapshot(names):
        pending = {name: clients[name].call_async(GetState.Request())
                   for name in names if clients[name].service_is_ready()}
        until = min(deadline, clock()+.25)
        while any(not future.done() for future in pending.values()) and clock() < until:
            ros.spin_once(node, timeout_sec=.02)
        states = {name: None for name in names}
        for name, future in pending.items():
            if future.done() and not future.cancelled() and future.exception() is None:
                response = future.result()
                if response is not None:
                    states[name] = response.current_state.id
            else:
                clients[name].remove_pending_request(future)
                future.cancel()
        return states

    policy, last_states = SettledPlan(), None
    while clock() < deadline:
        states = snapshot(NODES)
        if clock() >= deadline:
            diagnostic('LIFECYCLE_RECOVERY_NOT_SETTLED')
            return 2
        plan = policy.observe(states, clock())
        if plan is not None:
            diagnostic('LIFECYCLE_SETTLED_FOR_SHUTDOWN', states=states, plan=plan)
            break
        if states != last_states:
            diagnostic('LIFECYCLE_WAIT_RECOVERY', states=states)
            last_states = states
        spin_until(min(deadline, clock()+.2))  # At most five read-only batches/s.
    else:
        diagnostic('LIFECYCLE_RECOVERY_NOT_SETTLED')
        return 2

    for manager, names in GROUPS.items():
        if plan[manager] == 'FINALIZED':
            diagnostic('LIFECYCLE_ALREADY_FINALIZED', node=manager)
            continue
        client = managers[manager]
        if not client.service_is_ready() or clock() >= deadline:
            diagnostic('LIFECYCLE_UNAVAILABLE', node=manager)
            return 2
        request = ManageLifecycleNodes.Request()
        request.command = request.SHUTDOWN
        future = client.call_async(request)
        while not future.done() and clock() < deadline:
            ros.spin_once(node, timeout_sec=.02)
        if (not future.done() or future.cancelled() or future.exception() is not None
                or future.result() is None or not future.result().success):
            diagnostic('LIFECYCLE_NOT_CONFIRMED', node=manager)
            return 2
        while clock() < deadline:
            states = snapshot(names)
            if all(type(value) is int and value == 4 for value in states.values()):
                diagnostic('LIFECYCLE_SHUTDOWN_CONFIRMED', node=manager, states=states)
                break
            spin_until(min(deadline, clock()+.2))
        else:
            diagnostic('LIFECYCLE_FINALIZE_NOT_CONFIRMED', node=manager)
            return 2
    return 0
