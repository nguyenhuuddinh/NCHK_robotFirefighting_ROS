import json
from types import SimpleNamespace as N
import pytest
from operational_peer import SshPi,same_binding
from r5_pi_session import PiController

B=dict(session='session',secret='secret',packet='packet',boot_id='boot',runner_pid=123,runner_start_ticks=2,root='/home/pi/runs/session')


def controller():
    events=[];stop=[None];h={'io':True}
    p=PiController(B,lambda:events.append('START_IO'),lambda:events.append('BEGIN_STOP'),lambda:stop[0],lambda:h,0)
    return p,events,stop,h


def test_pi_clock_wait_no_ros_start_until_exact_start_then_seal_prevents_retry():
    p,e,s,h=controller();p.command('STATE',1);assert not e
    p.command('START',2);assert e==['START_IO']
    p.command('SEAL',3)
    with pytest.raises(ValueError):p.command('RETRY_PREARM',4)
    assert p.phase=='RUNNING_OBSERVE' and not p.report()['live']


def test_retry_must_release_all_prior_resources_before_restarting():
    p,e,s,h=controller();p.command('START',1);p.command('RETRY_PREARM',2);p.tick(3)
    assert e==['START_IO','BEGIN_STOP']
    s[0]=dict(remote_groups_exited=True,children_clean=True,forced=False);p.tick(4)
    assert e==['START_IO','BEGIN_STOP','START_IO']
    with pytest.raises(ValueError):p.command('RETRY_PREARM',5)


@pytest.mark.parametrize('field',['remote_groups_exited','children_clean','forced'])
def test_bad_cleanup_no_camera_retry_relaunch(field):
    p,e,s,h=controller();p.command('START',1);p.command('RETRY_PREARM',2)
    s[0]=dict(remote_groups_exited=True,children_clean=True,forced=False);s[0][field]=field=='forced';p.tick(3)
    assert p.phase=='DONE' and e.count('START_IO')==1


def test_duplicate_start_unknown_command_rejected_and_unready_seal_waits():
    p,e,s,h=controller();p.command('START',1)
    with pytest.raises(ValueError):p.command('START',2)
    with pytest.raises(ValueError):p.command('ARM',2)
    h['io']=False
    r=p.command('SEAL',3);assert r['seal_requested'] and not r['sealed'] and r['phase']=='IO_STARTING'


def test_repeated_stop_idempotent_and_wait360_timeout_exits_without_io():
    p,e,s,h=controller();p.tick(360);p.command('STOP',361);assert e==['BEGIN_STOP']
    s[0]=dict(remote_groups_exited=True,children_clean=True,forced=False);p.tick(362);assert p.phase=='DONE'


def test_ssh_success_is_not_remote_cleanup_proof_and_no_runtime_ssh_health_loop():
    calls=[];now=[0.]
    def run(argv,**kwargs):
        calls.append(argv);return N(returncode=0,stdout=json.dumps(dict(binding=B,state=dict(session=B['session'],phase='RUNNING_OBSERVE'))))
    peer=SshPi(B,'/pi/with space/operational_peer.py',run=run)
    result=peer.stop(clock=lambda:now[0],sleep=lambda s:now.__setitem__(0,now[0]+s))
    assert not result['confirmed'] and not result['local_SSH_exit_is_proof']
    assert sum("'STOP'" in a[-1] or '--command STOP' in a[-1] for a in calls)==1


def test_remote_done_uses_bound_group_child_proof():
    def run(argv,**kwargs):return N(returncode=0,stdout=json.dumps(dict(binding=B,state=dict(session=B['session'],phase='DONE',cleanup=dict(remote_groups_exited=True,children_clean=True,forced=False)))))
    r=SshPi(B,'/pi/helper',run=run).stop();assert r['confirmed'] and r['reason']=='REMOTE_EXIT_PROVEN'


@pytest.mark.parametrize('bad',['binding','session','request_error','ssh'])
def test_remote_stale_or_failed_request_not_accepted(bad):
    reply=dict(binding=dict(B),state=dict(session=B['session'],phase='WAIT_LAPTOP'))
    if bad=='binding':reply['binding']['runner_start_ticks']=99
    if bad=='session':reply['state']['session']='other'
    if bad=='request_error':reply['state']['request_error']='RETRY_AFTER_SEAL'
    def run(*a,**k):return N(returncode=1 if bad=='ssh' else 0,stdout=json.dumps(reply))
    with pytest.raises((ValueError,RuntimeError)):SshPi(B,'helper',run=run).request('START')


def test_partial_pi_start_failure_tracks_resources_and_enters_cleanup_immediately():
    events=[]
    def partial():events.append('ROBOT_STARTED');raise RuntimeError('health spawn failed')
    p=PiController(B,partial,lambda:events.append('CLEANUP_PARTIAL'),lambda:None,lambda:{},0)
    with pytest.raises(RuntimeError):p.command('START',1)
    assert p.phase=='STOPPING' and p.stop_started and p.error=='PI_IO_START_FAILED'
    assert events==['ROBOT_STARTED','CLEANUP_PARTIAL']


def test_remote_runner_failed_cannot_be_reported_clean_even_groups_exited():
    def run(argv,**kwargs):return N(returncode=0,stdout=json.dumps(dict(binding=B,state=dict(session=B['session'],phase='DONE',error='PI_RUNNER_FAILED',cleanup=dict(remote_groups_exited=True,children_clean=True,forced=False)))))
    r=SshPi(B,'/pi/helper',run=run).stop();assert not r['confirmed'] and r['runner_error']=='PI_RUNNER_FAILED'


def test_startup_state_transport_retry_bounded_without_accepting_unknown_health():
    from operational_peer import PeerTransportError
    peer=SshPi(B,'helper');calls=[]
    def request(cmd):
        calls.append(cmd)
        if len(calls)<3:raise PeerTransportError('SSH timeout')
        return dict(phase='WAIT_LAPTOP')
    peer.request=request
    assert peer.startup('STATE',sleep=lambda s:None)['phase']=='WAIT_LAPTOP'
    assert calls==['STATE']*3


def test_startup_three_failed_queries_still_block_and_protocol_error_not_retried():
    from operational_peer import PeerTransportError
    peer=SshPi(B,'helper');calls=[]
    def request(cmd):calls.append(cmd);raise PeerTransportError('timeout')
    peer.request=request
    with pytest.raises(PeerTransportError):peer.startup('STATE',sleep=lambda s:None)
    assert calls==['STATE']*3
    calls.clear()
    def protocol(cmd):calls.append(cmd);raise ValueError('wrong bridge session')
    peer.request=protocol
    with pytest.raises(ValueError):peer.startup('STATE',sleep=lambda s:None)
    assert calls==['STATE']


def test_lost_start_ack_recovered_from_bound_state_without_start_replay():
    from operational_peer import PeerTransportError
    peer=SshPi(B,'helper');calls=[]
    def request(cmd):
        calls.append(cmd)
        if cmd=='START':raise PeerTransportError('reply lost')
        return dict(phase='IO_STARTING',retries=0)
    peer.request=request;assert peer.startup('START',sleep=lambda s:None)['phase']=='IO_STARTING'
    assert calls==['START','STATE']


def test_lost_start_ack_not_applied_never_claims_started():
    from operational_peer import PeerTransportError
    peer=SshPi(B,'helper')
    def request(cmd):
        if cmd=='START':raise PeerTransportError('reply lost')
        return dict(phase='WAIT_LAPTOP',retries=0)
    peer.request=request
    with pytest.raises(PeerTransportError):peer.startup('START',sleep=lambda s:None)
