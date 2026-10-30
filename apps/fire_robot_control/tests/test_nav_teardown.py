"""No invalid transitions during recovery and no ACK-only cleanup success."""
from concurrent.futures import Future
from types import SimpleNamespace as S
import unittest

from services.nav_teardown import GROUPS, NODES, SettledPlan, drain_lifecycle


class PlanTests(unittest.TestCase):
    def test_all_nodes_must_stay_ready_before_any_shutdown(self):
        policy = SettledPlan()
        for at in (0., .2, .4):
            self.assertIsNone(policy.observe(dict.fromkeys(NODES, 3), at))
        self.assertEqual(set(policy.observe(dict.fromkeys(NODES, 3), .6).values()), {'SHUTDOWN'})

    def test_unconfigured_transitioning_missing_and_bool_states_never_admit(self):
        for bad in (None, 1, 2, 10, 13, True):
            policy = SettledPlan()
            states = dict.fromkeys(NODES, 3)
            states[NODES[0]] = bad
            for at in (0., .2, .4, .6, .8):
                self.assertIsNone(policy.observe(states, at))
        self.assertIsNone(SettledPlan().observe({}, 0.))

    def test_missed_samples_cannot_accumulate_stability(self):
        policy = SettledPlan()
        self.assertIsNone(policy.observe(dict.fromkeys(NODES, 3), 0.))
        self.assertIsNone(policy.observe(dict.fromkeys(NODES, 3), 1.))

    def test_clock_discontinuity_cannot_admit(self):
        for at in (-1., float('nan'), float('inf')):
            policy = SettledPlan()
            policy.observe(dict.fromkeys(NODES, 3), 0.)
            self.assertIsNone(policy.observe(dict.fromkeys(NODES, 3), at))

    def test_finalized_groups_are_not_transitioned_again(self):
        states = dict.fromkeys(NODES, 3)
        for name in GROUPS['lifecycle_manager_navigation']:
            states[name] = 4
        policy = SettledPlan()
        for at in (0., .2, .4): policy.observe(states, at)
        self.assertEqual(policy.observe(states, .6), {
            'lifecycle_manager_navigation': 'FINALIZED', 'lifecycle_manager_localization': 'SHUTDOWN'})


class LifecycleBus:
    def __init__(self, *, recovery_until=0., available=True, stuck=False, ack=True, finalize=True, nav_finalized=False):
        self.now = 0.
        self.recovery_until, self.available, self.stuck = recovery_until, available, stuck
        self.ack, self.finalize = ack, finalize
        self.final = set(GROUPS['lifecycle_manager_navigation']) if nav_finalized else set()
        self.commands, self.events, self.cancelled = [], [], 0

    def spin_once(self, node, timeout_sec): self.now += max(.001, timeout_sec)
    def diagnostic(self, event, **fields): self.events.append((self.now, event, fields))
    def create_client(self, typ, path):
        bus = self
        name = path.split('/')[1]
        class Client:
            def service_is_ready(self): return bus.available
            def remove_pending_request(self, future): bus.cancelled += 1
            def call_async(self, request):
                future = Future()
                if bus.stuck: return future
                if path.endswith('/get_state'):
                    state = 4 if name in bus.final else 1 if bus.now < bus.recovery_until else 3
                    future.set_result(S(current_state=S(id=state)))
                else:
                    bus.commands.append((bus.now, name, request.command))
                    if bus.now < bus.recovery_until:
                        raise AssertionError('Shutdown requested during reset')
                    if bus.finalize and bus.ack: bus.final.update(GROUPS[name])
                    future.set_result(S(success=bus.ack))
                return future
        return Client()

    def run(self, deadline=3.):
        return drain_lifecycle(self, self, deadline, self.diagnostic, clock=lambda: self.now)


class LifecycleDrainTests(unittest.TestCase):
    def test_recovery_finishes_before_shutdown_and_all_final_states_are_checked(self):
        bus = LifecycleBus(recovery_until=.8)
        self.assertEqual(bus.run(), 0)
        self.assertEqual([row[1] for row in bus.commands], list(GROUPS))
        self.assertGreaterEqual(bus.commands[0][0], 1.3)
        self.assertEqual(bus.final, set(NODES))
        self.assertEqual(sum(event == 'LIFECYCLE_SHUTDOWN_CONFIRMED' for _,event,_ in bus.events), 2)

    def test_ack_without_finalized_nodes_is_not_success(self):
        bus = LifecycleBus(finalize=False)
        self.assertEqual(bus.run(), 2)
        self.assertEqual(len(bus.commands), 1)
        self.assertEqual(bus.events[-1][1], 'LIFECYCLE_FINALIZE_NOT_CONFIRMED')

    def test_false_ack_never_becomes_success(self):
        bus = LifecycleBus(ack=False)
        self.assertEqual(bus.run(), 2)
        self.assertEqual(bus.events[-1][1], 'LIFECYCLE_NOT_CONFIRMED')

    def test_unavailable_services_never_trigger_a_transition(self):
        bus = LifecycleBus(available=False)
        self.assertEqual(bus.run(), 2)
        self.assertEqual(bus.commands, [])

    def test_stuck_queries_are_cancelled_and_bounded(self):
        bus = LifecycleBus(stuck=True)
        self.assertEqual(bus.run(deadline=1.), 2)
        self.assertEqual(bus.commands, [])
        self.assertGreater(bus.cancelled, 0)
        self.assertLess(bus.now, 1.05)

    def test_permanent_reset_never_triggers_a_shutdown_transition(self):
        bus = LifecycleBus(recovery_until=10.)
        self.assertEqual(bus.run(), 2)
        self.assertEqual(bus.commands, [])

    def test_already_finalized_navigation_is_not_transitioned_again(self):
        bus = LifecycleBus(nav_finalized=True)
        self.assertEqual(bus.run(), 0)
        self.assertEqual([row[1] for row in bus.commands], ['lifecycle_manager_localization'])

    def test_all_finalized_nodes_are_not_transitioned_again(self):
        bus = LifecycleBus()
        bus.final.update(NODES)
        self.assertEqual(bus.run(), 0)
        self.assertEqual(bus.commands, [])


if __name__ == '__main__': unittest.main()
