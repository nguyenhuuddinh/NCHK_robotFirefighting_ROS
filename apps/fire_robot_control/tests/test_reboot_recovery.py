"""Reboot recovery uses synthetic proofs; never starts devices or SSH."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import uuid
from unittest.mock import patch

from services import recovery, recovery_probe as probe, stop_recovery_runner as stop
import test_stop_recovery as stop_fixture


class RebootBaseline(unittest.TestCase):
    setUp = stop_fixture.StopRecoveryTests.setUp
    tearDown = stop_fixture.StopRecoveryTests.tearDown
    new_proof = stop_fixture.StopRecoveryTests.new_proof
    def test_prepare_accepts_new_boot_idle_proof_without_claiming_stop(self):
        idle = dict(current_boot_id=str(uuid.uuid4()), mode='NEW_BOOT_QUIESCENT_ONLY')
        with patch.object(stop.probe, 'absent'), patch.object(stop.probe, 'no_robot_processes'), \
             patch.object(recovery, 'remote_idle_proof', return_value=idle, create=True), \
             patch.object(recovery, 'remote_proof', side_effect=ValueError('RECOVERY_PI_BOOT_CHANGED')):
            result = stop.prepare(self.root, lock_path=self.lock)
        self.assertEqual(result['marker_sha256'], self.digest)
        self.assertEqual(result['pi'], idle)
        self.assertEqual((self.root/'pending.json').read_bytes(), self.raw)

    def test_nested_boot_error_is_visible_without_transport_contents(self):
        text = recovery.failure_message(ValueError('RECOVERY_PI_CHECK_FAILED:RECOVERY_PI_BOOT_CHANGED'))
        self.assertIn('RECOVERY_PI_BOOT_CHANGED', text)
        self.assertIn('khởi động lại', text)

    def test_new_stop_audit_then_archive_on_current_boot_keeps_old_failure(self):
        folder=self.new_proof();desktop=json.loads((folder/'desktop_receipt.json').read_text())
        calls=[];old=(self.old/'laptop/receipt.json').read_bytes()
        with patch.object(probe,'absent'),patch.object(probe,'no_robot_processes'):
            result=stop.audit_new_stop(self.root,folder,remote=lambda b:dict(binding=b),
                idle_remote=lambda b,expected_boot:calls.append(expected_boot) or dict(current_boot_id=expected_boot),lock_path=self.lock)
            recovery.archive_verified(self.root,result,'4'*32,lock_path=self.lock)
        self.assertEqual(calls,[desktop['recovery_boot_id']]);self.assertFalse((self.root/'pending.json').exists())
        self.assertEqual((self.old/'laptop/receipt.json').read_bytes(),old)
        self.assertIn('new_stop_proof',result)

    def test_missing_stop_cleanup_or_boot_pin_never_reaches_network_or_archive(self):
        folder=self.new_proof();dp=folder/'desktop_receipt.json';rp=folder/'laptop/receipt.json'
        desktop=json.loads(dp.read_text());receipt=json.loads(rp.read_text())
        for target,key,value in [('desktop','robot_stop_confirmed',False),('desktop','cleanup_clean',False),
                ('desktop','recovery_boot_id',None),('desktop','recovery_boot_id',str(uuid.uuid4())),
                ('desktop','recovery_marker_sha256','b'*64),('receipt','session',str(uuid.uuid4()))]:
            with self.subTest(target=target,key=key):
                d=copy.deepcopy(desktop);r=copy.deepcopy(receipt);(d if target=='desktop' else r)[key]=value
                dp.write_text(json.dumps(d));rp.write_text(json.dumps(r))
                with self.assertRaises(ValueError):
                    stop.audit_new_stop(self.root,folder,remote=lambda _:self.fail('network reached'),lock_path=self.lock)
                self.assertEqual((self.root/'pending.json').read_bytes(),self.raw)

    def test_second_reboot_after_new_stop_or_live_owner_keeps_pending(self):
        folder=self.new_proof()
        for cause in ('RECOVERY_PI_BOOT_CHANGED','RECOVERY_ROBOT_PROCESS_OR_SNAPSHOT_UNKNOWN'):
            with self.subTest(cause=cause),patch.object(probe,'absent'),patch.object(probe,'no_robot_processes'):
                def idle(*args,**kwargs):raise ValueError(cause)
                with self.assertRaisesRegex(ValueError,cause):
                    stop.audit_new_stop(self.root,folder,remote=lambda _:dict(ok=True),idle_remote=idle,lock_path=self.lock)
                self.assertEqual((self.root/'pending.json').read_bytes(),self.raw)

    def test_reusing_old_pi_session_is_not_fresh_stop_evidence(self):
        folder=self.new_proof();p=folder/'laptop/receipt.json';r=json.loads(p.read_text())
        r['remote']['binding']['session']=self.row['remote']['binding']['session'];p.write_text(json.dumps(r))
        with patch.object(probe,'absent'),patch.object(probe,'no_robot_processes'):
            with self.assertRaisesRegex(ValueError,'NEW_STOP_NOT_CONFIRMED'):
                stop.audit_new_stop(self.root,folder,remote=lambda _:self.fail('network'),lock_path=self.lock)


class IdlePiProbeTests(unittest.TestCase):
    def setUp(self):
        self.binding=dict(session=str(uuid.uuid4()),packet='a'*64,boot_id=str(uuid.uuid4()),runner_pid=80,
            runner_start_ticks=99,root='/home/pi/qa_logs/r5_automatic_20261029_120000')
        self.current=str(uuid.uuid4())

    def fixture(self,boots=None):
        from contextlib import ExitStack
        from proof_fixture import clean_remote
        stack=ExitStack();self.addCleanup(stack.close)
        stack.enter_context(patch.object(Path,'is_symlink',return_value=False))
        stack.enter_context(patch.object(Path,'resolve',lambda p:p))
        stack.enter_context(patch.object(Path,'read_text',side_effect=boots or [self.current,self.current]))
        pairing=dict(self.binding,secret='private-never-return')
        done=dict(session=self.binding['session'],phase='DONE',live=False,
                  error='PI_SEAL_HEALTH_TIMEOUT',cleanup=clean_remote({})['receipt'])
        reads=stack.enter_context(patch.object(probe,'read_object',side_effect=lambda p:(pairing if p.name=='pairing.json' else done,'c'*64)))
        locks=stack.enter_context(patch.object(probe,'lock_existing',return_value=123))
        stack.enter_context(patch.object(probe.os,'close'))
        absent=stack.enter_context(patch.object(probe,'absent'))
        quiet=stack.enter_context(patch.object(probe,'no_robot_processes'))
        return pairing,done,locks,absent,quiet

    def test_reboot_needs_quiescence_and_never_claims_stop_or_reads_old_done(self):
        pairing,done,locks,absent,quiet=self.fixture()
        result=probe.probe_pi_idle(self.binding)
        self.assertEqual(result['mode'],'NEW_BOOT_QUIESCENT_ONLY')
        self.assertEqual(result['current_boot_id'],self.current)
        self.assertNotIn('stop_confirmed',result);self.assertNotIn('cleanup',result)
        self.assertNotIn('private-never-return',json.dumps(result));absent.assert_not_called()
        self.assertEqual([call.args[0].name for call in probe.read_object.call_args_list],['pairing.json'])
        self.assertIn('[c]amsense_x1_node',quiet.call_args.kwargs['pattern']);locks.assert_called_once()

    def test_same_boot_retains_cleanup_and_pid_reuse_checks(self):
        self.current=self.binding['boot_id'];pairing,done,locks,absent,quiet=self.fixture([self.current]*8)
        result=probe.probe_pi_idle(self.binding)
        self.assertEqual(result['mode'],'SAME_BOOT_CLEANUP');self.assertIn('historical_cleanup',result)
        absent.side_effect=ValueError('RECOVERY_PROCESS_STILL_LIVE')
        with self.assertRaisesRegex(ValueError,'STILL_LIVE'):probe.probe_pi_idle(self.binding)

    def test_wrong_pairing_or_unknown_owner_or_busy_lock_is_rejected(self):
        pairing,done,locks,absent,quiet=self.fixture([self.current]*12)
        for key in self.binding:
            with self.subTest(binding_field=key):
                original=pairing[key];pairing[key]='foreign'
                with self.assertRaisesRegex(ValueError,'BINDING_MISMATCH'):probe.probe_pi_idle(self.binding)
                pairing[key]=original
        quiet.side_effect=ValueError('RECOVERY_ROBOT_PROCESS_OR_SNAPSHOT_UNKNOWN')
        with self.assertRaisesRegex(ValueError,'SNAPSHOT_UNKNOWN'):probe.probe_pi_idle(self.binding)
        quiet.side_effect=None;locks.side_effect=BlockingIOError('busy')
        with self.assertRaises(BlockingIOError):probe.probe_pi_idle(self.binding)

    def test_boot_changes_during_probe_or_since_prepare_are_rejected(self):
        self.fixture([self.current,str(uuid.uuid4()),self.current])
        with self.assertRaisesRegex(ValueError,'BOOT_CHANGED'):probe.probe_pi_idle(self.binding)
        with self.assertRaisesRegex(ValueError,'BOOT_CHANGED'):probe.probe_pi_idle(self.binding,expected_boot=str(uuid.uuid4()))


class IdleTransportTests(unittest.TestCase):
    def setUp(self):
        self.binding=dict(session=str(uuid.uuid4()),packet='a'*64,boot_id=str(uuid.uuid4()),runner_pid=80,
            runner_start_ticks=99,root='/home/pi/qa_logs/r5_automatic_20261029_120000')
        self.boot=str(uuid.uuid4())

    def request(self,changes=None,error=None):
        import ast
        from types import SimpleNamespace
        def run(argv,**kw):
            self.assertIn('StrictHostKeyChecking=yes',argv);self.assertEqual(kw['timeout'],18)
            call=ast.parse(kw['input'].splitlines()[-1]).body[0].value
            nonce=ast.literal_eval(call.args[-1])
            proof=dict(binding=self.binding,current_boot_id=self.boot,mode='NEW_BOOT_QUIESCENT_ONLY',
                request_nonce=nonce,pairing_sha256='c'*64,processes_absent=True,common_lock_free=True,boot_stable=True)
            proof.update(changes or {})
            return SimpleNamespace(returncode=0,stdout=json.dumps(dict(ok=False,error=error) if error else dict(ok=True,proof=proof)))
        with patch.object(recovery,'pi_host',return_value='10.0.0.1'):
            return recovery.remote_idle_proof(self.binding,expected_boot=self.boot,run=run)

    def test_challenge_bound_response_is_accepted_only_for_expected_boot(self):
        self.assertEqual(self.request()['current_boot_id'],self.boot)
        for field,value in [('request_nonce','replay'),('binding',{}),('current_boot_id',str(uuid.uuid4())),
                ('processes_absent',False),('common_lock_free',False),('boot_stable',False),
                ('mode','SAME_BOOT_CLEANUP'),('pairing_sha256','bad')]:
            with self.subTest(field=field),self.assertRaises(ValueError):self.request({field:value})

    def test_remote_errors_are_protocol_only_and_unknown_text_not_echoed(self):
        with self.assertRaisesRegex(ValueError,'RECOVERY_PI_BOOT_CHANGED'):self.request(error='RECOVERY_PI_BOOT_CHANGED')
        with self.assertRaisesRegex(ValueError,'^RECOVERY_PI_CHECK_FAILED$'):self.request(error='private password text')

    def test_same_boot_drain_proof_cannot_omit_original_cleanup(self):
        self.boot=self.binding['boot_id']
        with self.assertRaisesRegex(ValueError,'PROOF_INVALID'):self.request(dict(mode='SAME_BOOT_CLEANUP'))


class LiveRecoveryErrors(unittest.TestCase):
    def test_real_worker_preserves_recovery_cause_and_starts_no_io(self):
        from services.live_session import LiveSession
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);events=[]
            for name in ('candidate','runtime','out','maps'):(p/name).mkdir()
            runner=LiveSession(candidate=p/'candidate',packet='a'*64,runtime=p/'runtime',out=p/'out',
                map_root=p/'maps',map_roots=[p/'maps'],kind='verify_stop',selected_map=None,emit=events.append)
            with patch.object(stop,'prepare',side_effect=ValueError('RECOVERY_PI_CHECK_FAILED:RECOVERY_PI_BOOT_CHANGED')), \
                 patch.object(runner,'_launch_pi') as launch:
                runner.run()
            launch.assert_not_called();last=events[-1]
            self.assertIn('RECOVERY_PI_BOOT_CHANGED',last['error']);self.assertFalse(last['session_was_started'])
            receipt=json.loads((p/'out/desktop_receipt.json').read_text())
            self.assertIn('RECOVERY_PI_BOOT_CHANGED',receipt['primary_error'])

    def test_boot_change_before_start_never_launches_laptop_or_io(self):
        import test_live_session_fake as fake
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);events=[]
            for name in ('candidate','runtime','out','maps'):(p/name).mkdir()
            runner=fake.FakeLiveSession(candidate=p/'candidate',packet='a'*64,runtime=p/'runtime',out=p/'out',
                map_root=p/'maps',map_roots=[p/'maps'],kind='verify_stop',selected_map=None,emit=events.append,pi_receipt=p/'receipt')
            prepared=dict(marker_sha256='f'*64,pi=dict(current_boot_id=str(uuid.uuid4())))
            class Peer:
                def __init__(self,*args):pass
                def startup(self,*args):return dict(phase='WAIT_LAPTOP')
                def stop(self):return dict(confirmed=True)
            with patch.object(stop,'prepare',return_value=prepared), \
                 patch('services.live_session.subprocess.run',return_value=SimpleNamespace(returncode=0)), \
                 patch('services.live_session.validate_pi_receipt'), \
                 patch('services.live_session.PairingTransfer',fake.FakeTransfer), \
                 patch('operational_peer.SshPi',Peer),patch.object(runner,'_spawn_laptop') as spawn:
                runner.run()
            spawn.assert_not_called();self.assertIsNone(runner.local)
            self.assertIn('RECOVERY_PI_BOOT_CHANGED',events[-1]['error'])
            self.assertEqual(runner.recovery_boot_id,prepared['pi']['current_boot_id'])
