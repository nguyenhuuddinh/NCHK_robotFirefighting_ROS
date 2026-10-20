"""Real temporary ledger I/O, bounded quotas and crash/restart boundaries."""

from copy import deepcopy
import json

import pytest

from fire_robot_mission.shadow_budget import LIMITS, ShadowBudgetStore


GOAL = 'ab' * 16


def sid(n=1):
    return 'fire:' + 'cd' * 16 + ':' + str(n)


@pytest.fixture
def store(tmp_path):
    result = ShadowBudgetStore(str(tmp_path / 'budget.json'), initialize=True)
    yield result
    result.close()


def complete(store, identity):
    store.mark_completion(identity)
    store.finish_completion(identity)


def test_missing_requires_explicit_initialization_and_existing_state_never_reinitializes(tmp_path):
    path = str(tmp_path / 'budget.json')
    with pytest.raises(ValueError, match='initialization'):
        ShadowBudgetStore(path)
    one = ShadowBudgetStore(path, initialize=True)
    one.reserve_session(sid(), GOAL)
    saved = deepcopy(one.state)
    one.close()
    two = ShadowBudgetStore(path, initialize=True)
    try:
        assert two.state == saved
        assert two.state['pending']['session_id'] == sid()
    finally:
        two.close()


def test_two_process_owners_cannot_use_same_record(store):
    with pytest.raises(BlockingIOError):
        ShadowBudgetStore(str(store.path))
    store.close()
    again = ShadowBudgetStore(str(store.path))
    again.close()


def test_three_sessions_reserve_total_limits_and_no_refund_on_completion(store):
    for n in range(1, 4):
        store.reserve_session(sid(n), GOAL if n != 2 else None)
        assert store.state['used']['sessions'] == n
        assert store.state['used']['pump_ms'] == n * 5000
        assert store.state['used']['approach_um'] == LIMITS['approach_um'] * n // 3
        complete(store, sid(n))
    assert store.state['used'] == LIMITS
    with pytest.raises(ValueError):
        store.reserve_session(sid(4), None)
    assert store.snapshot()['actuation_authorized'] is False
    assert store.snapshot()['planning_only'] is True


def test_restart_keeps_marked_pending_and_recovery_cannot_change_goal_or_refund(store):
    store.reserve_session(sid(), GOAL)
    store.mark_completion(sid())
    used = deepcopy(store.state['used'])
    path = str(store.path)
    store.close()
    again = ShadowBudgetStore(path)
    try:
        with pytest.raises(ValueError):
            again.recover(sid(2), None)
        again.recover(sid(2), GOAL)
        assert again.state['used'] == used
        assert again.state['pending']['origin_session_id'] == sid()
        assert again.state['pending']['completion_marked'] is False
        assert again.state['recoveries'] == 1
        complete(again, sid(2))
        assert again.state['used'] == used
    finally:
        again.close()


def test_reset_only_after_explicit_probe_and_verified_completion(store):
    store.reserve_session(sid(), None)
    complete(store, sid())
    encounter = store.state['encounter_id']
    store.begin_reset_probe(sid(2))
    assert store.state['used']['sessions'] == 1
    store.mark_completion(sid(2))
    with pytest.raises(ValueError):
        store.finish_completion(sid(2))
    assert store.state['encounter_id'] == encounter
    store.finish_completion(sid(2), reset_verified=True)
    assert store.state['encounter_id'] != encounter
    assert store.state['used'] == dict.fromkeys(LIMITS, 0)


@pytest.mark.parametrize('field,value', [
    ('schema_version', True), ('used', {}), ('recoveries', 4),
    ('recoveries', True), ('encounter_id', 'invalid'), ('pending', {}),
    ('last_completed', 'old'), ('limits', {}),
])
def test_corrupt_records_fail_startup_instead_of_resetting(store, field, value):
    state = deepcopy(store.state)
    path = store.path
    store.close()
    state[field] = value
    path.write_text(json.dumps(state))
    with pytest.raises(ValueError):
        ShadowBudgetStore(str(path), initialize=True)


def test_inconsistent_reservations_and_truncated_json_are_not_repaired(store):
    path = store.path
    state = deepcopy(store.state)
    store.close()
    state['used']['sessions'] = 1
    path.write_text(json.dumps(state))
    with pytest.raises(ValueError):
        ShadowBudgetStore(str(path))
    path.write_text('{')
    with pytest.raises(ValueError):
        ShadowBudgetStore(str(path), initialize=True)


def test_atomic_replace_failure_preserves_old_record_and_propagates(store, monkeypatch):
    from fire_robot_mission import shadow_budget
    before = store.path.read_bytes()
    def fail(*args):
        raise OSError('disk failure')
    monkeypatch.setattr(shadow_budget.os, 'replace', fail)
    with pytest.raises(OSError, match='disk failure'):
        store.reserve_session(sid(), GOAL)
    assert store.path.read_bytes() == before
    assert store.state['used']['sessions'] == 0
    assert list(store.path.parent.glob('budget.json.*')) == [store.path.parent / 'budget.json.lock']
    with pytest.raises(RuntimeError, match='uncertain'):
        store.reserve_session(sid(), GOAL)


def test_recovery_counter_saturates_without_refunding_or_deadlocking_recovery(store):
    store.reserve_session(sid(), GOAL)
    for n in range(2, 8):
        store.recover(sid(n), GOAL)
    complete(store, sid(7))
    assert store.state['recoveries'] == 3
    assert store.state['used']['sessions'] == 1
    store.begin_probe(sid(8), None, purpose='RECOVERY')
    assert store.state['recoveries'] == 3
    assert store.state['used']['sessions'] == 1


def test_rejects_relative_path_and_state_or_lock_symlinks(tmp_path):
    with pytest.raises(ValueError):
        ShadowBudgetStore('relative.json', initialize=True)
    target = tmp_path / 'real'
    target.write_text('{}')
    state = tmp_path / 'state'
    state.symlink_to(target)
    with pytest.raises(ValueError):
        ShadowBudgetStore(str(state), initialize=True)
    lock_state = tmp_path / 'other'
    (tmp_path / 'other.lock').symlink_to(target)
    with pytest.raises(OSError):
        ShadowBudgetStore(str(lock_state), initialize=True)


@pytest.mark.parametrize('identity', ['fire:x:1', 'fire:' + '0' * 32 + ':1',
                                      'fire:' + 'ab' * 16 + ':01', 'fire:' + 'ab' * 16 + ':0'])
def test_invalid_session_namespace_or_sequence_cannot_enter_ledger(store, identity):
    with pytest.raises(ValueError):
        store.reserve_session(identity, None)
    assert store.state['used']['sessions'] == 0


def test_closed_store_cannot_commit_without_its_lock(store):
    before = store.path.read_bytes()
    store.close()
    with pytest.raises(RuntimeError, match='closed'):
        store.reserve_session(sid(), None)
    assert store.path.read_bytes() == before
