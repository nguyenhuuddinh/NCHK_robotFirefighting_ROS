"""R5.2 context delta; no ROS, no synthetic canceled goal, no old suite rerun."""
import pytest
from fire_robot_mission.operational_cycle import CycleRegistry, NavigationSnapshot

OLD = 'ab'*16
NEW = 'cd'*16
CYCLE = 'ef'*16


def begin(rows=()):
    registry=CycleRegistry(lambda:CYCLE)
    identity=registry.begin(rows,1.,1.,armed=True,fire_confirmed=True)
    return registry,identity


@pytest.mark.parametrize('rows', [[], [(OLD,4)], [(OLD,5)], [(OLD,6)]])
def test_fire_without_any_active_or_successful_goal_has_own_cycle(rows):
    registry,identity=begin(rows)
    assert identity.cycle_id==CYCLE and identity.nav_goal_uuid is None
    assert NavigationSnapshot.parse(rows,1.).rows==tuple(rows)
    assert registry.report(1.)['motion_authorized'] is False


@pytest.mark.parametrize('status', [1,2,3])
def test_fire_binds_real_active_goal_and_distinct_cycle(status):
    registry,identity=begin([(OLD,status)])
    assert identity.nav_goal_uuid==OLD and identity.cycle_id!=OLD
    assert registry.retirement_request()==dict(cycle_id=CYCLE,nav_goal_uuid=OLD)


@pytest.mark.parametrize('rows', [[(OLD,0)],[(OLD,True)],[(OLD,7)],[(OLD,2),(NEW,1)],
    [(OLD,4),(OLD,4)],[('0'*32,2)],[('invalid',2)],[[OLD]]])
def test_bad_or_ambiguous_navigation_never_becomes_no_goal(rows):
    with pytest.raises(ValueError):begin(rows)


def test_stale_or_absent_status_is_not_no_active_goal_evidence():
    registry=CycleRegistry(lambda:CYCLE)
    with pytest.raises(ValueError,match='STALE'):
        registry.begin([],1.,1.4,armed=True,fire_confirmed=True)
    with pytest.raises(ValueError,match='INVALID'):
        registry.begin(None,1.4,1.4,armed=True,fire_confirmed=True)
    assert registry.current is None


def test_future_status_and_unarmed_fire_cannot_start_cycle():
    registry=CycleRegistry(lambda:CYCLE)
    with pytest.raises(ValueError,match='STALE'):
        registry.begin([],2.,1.,armed=True,fire_confirmed=True)
    with pytest.raises(ValueError,match='PERMISSION'):
        registry.begin([],1.,1.,armed=False,fire_confirmed=True)
    assert registry.current is None


def test_goal_change_cannot_replace_cycle_or_budget():
    registry,identity=begin([(OLD,2)])
    assert registry.budget.request(1.1,admission_ready=True)
    registry.budget.observe(CYCLE,.12,2.)
    with pytest.raises(ValueError,match='NEW_GOAL'):
        registry.begin([(OLD,5),(NEW,2)],2.,2.,armed=True,fire_confirmed=True)
    assert registry.current is identity and registry.budget.distance_m==.12


def test_target_loss_reacquire_and_terminal_event_do_not_reset_I():
    registry,identity=begin([(OLD,2)])
    assert registry.budget.request(1.1,admission_ready=True)
    registry.budget.observe(CYCLE,.12,2.)
    with pytest.raises(ValueError,match='PERMISSION'):
        registry.begin([(OLD,5)],3.,3.,armed=True,fire_confirmed=False)
    assert registry.begin([(OLD,5)],7.,7.,armed=True,fire_confirmed=True) is identity
    assert not registry.budget.request(7.,admission_ready=True)
    report=registry.report(7.)
    assert report['budget']['remaining_s']==pytest.approx(9.1)
    assert report['budget']['remaining_m']==pytest.approx(.18)
    assert report['requires_stop_release_handshake'] is True


def test_initial_I_not_ready_is_rejected_not_queued():
    registry,_=begin()
    assert not registry.budget.request(1.1,admission_ready=False)
    assert registry.budget.report(4.)['requested_at'] is None
    assert registry.budget.request(4.,admission_ready=True)
    assert registry.budget.report(4.)['remaining_s']==15.


@pytest.mark.parametrize('distance,at', [(.3,2.),(.12,16.1)])
def test_cap_and_wait_time_lock_forward_without_reissuing_budget(distance,at):
    registry,_=begin()
    registry.budget.request(1.1,admission_ready=True)
    registry.budget.observe(CYCLE,distance,at)
    report=registry.budget.report(at)
    assert report['exhausted'] and not report['forward_budget_available']
    assert not registry.budget.request(at,admission_ready=True)
    assert registry.budget.report(at+1)['exhausted']


@pytest.mark.parametrize('fault', ['uuid','distance_reset','nan','clock'])
def test_bad_progress_latches_fault_instead_of_fresh_budget(fault):
    registry,_=begin()
    registry.budget.request(1.1,admission_ready=True)
    registry.budget.observe(CYCLE,.12,2.)
    uid=NEW if fault=='uuid' else CYCLE
    value=.11 if fault=='distance_reset' else float('nan') if fault=='nan' else .12
    at=1.9 if fault=='clock' else 2.1
    assert not registry.budget.observe(uid,value,at)
    assert registry.budget.report(3.)['fault']
    assert not registry.budget.report(3.)['forward_budget_available']


def test_progress_before_I_is_not_hidden_as_normal_motion():
    registry,_=begin()
    assert not registry.budget.observe(CYCLE,.01,1.1)
    assert registry.budget.report(1.1)['fault']=='IR_PROGRESS_BEFORE_I'


def test_factory_must_not_fake_nav_goal_as_cycle_id():
    registry=CycleRegistry(lambda:OLD)
    with pytest.raises(ValueError,match='CYCLE_ID'):
        registry.begin([(OLD,5)],1.,1.,armed=True,fire_confirmed=True)
    assert registry.current is None
