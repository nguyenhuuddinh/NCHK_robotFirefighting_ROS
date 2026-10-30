"""Bounded user-action pairs on the actual controller, inert session workers."""
import itertools

import pytest

import test_supervisor_integration as controller_fixture


@pytest.mark.parametrize('stage', ['mapping_start', 'mapping', 'saving', 'fire_start', 'running'])
def test_action_pairs_do_not_poison_clean_stop_or_next_scan(stage):
    actions = ('SCAN', 'FIRE', 'SAVE', 'SELECT', 'X', 'STOP', 'RECHECK', 'VERIFY_STOP')
    for first, second in itertools.product(actions, repeat=2):
        fixture = controller_fixture.ControllerTests()
        fixture.setUp()
        try:
            c = fixture.ctrl
            c.action('SELECT', str(fixture.maps/'original.yaml'))
            c.action('FIRE' if stage in ('fire_start', 'running') else 'SCAN')
            worker = c.session
            if stage in ('mapping', 'saving'):
                worker.stage('MAPPING'); c.tick()
            elif stage == 'running':
                worker.stage('READY_GATE2'); c.tick()
            if stage == 'saving':
                c.action('SAVE', 'before_stop')
            for event in (first, second):
                argument = 'during_test' if event == 'SAVE' else str(fixture.maps/'original.yaml') if event == 'SELECT' else 'X' if event == 'X' else None
                c.action('KEY' if event == 'X' else event, argument)
                assert c.session is worker, (stage, first, second, c.status())
                assert c.phase != 'BLOCKED', (stage, first, second, c.status())
            c.action('STOP')
            worker.finish(stop=True, clean=True)
            c.tick()
            assert c.phase == 'IDLE' and c.session is None, (stage, first, second, c.status())
            assert not (fixture.state/'pending.json').exists()
            # Previous session callbacks cannot alter the newly launched scan.
            c.action('SCAN')
            next_worker = c.session
            worker.stage('READY_GATE2'); worker.finish(stop=False, clean=False)
            c.tick()
            assert c.session is next_worker and c.phase != 'BLOCKED'
            c.action('STOP'); next_worker.finish(stop=True, clean=True); c.tick()
            assert c.phase == 'IDLE'
        finally:
            fixture.tearDown()
