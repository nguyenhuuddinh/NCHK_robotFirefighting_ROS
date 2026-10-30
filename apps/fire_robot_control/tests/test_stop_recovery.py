"""Explicit STOP-only workflow, old pending never replaced by a new start."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import time
import unittest
import uuid
from unittest.mock import patch
from services import recovery
from services import stop_recovery_runner as stop
from services.supervisor import SessionController
from proof_fixture import clean_local,clean_remote,group,children
APP=Path(__file__).resolve().parents[1]
EVIDENCE=APP/'docs/qa_evidence/startup_block_215926_220156_20261029/raw_laptop/mapping'

class StopRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.old=self.root/('session_'+'1'*32);(self.old/'laptop').mkdir(parents=True)
        self.desktop=json.loads((EVIDENCE/'desktop_receipt.json').read_text())
        self.row=json.loads((EVIDENCE/'laptop/receipt.json').read_text())
        (self.old/'desktop_receipt.json').write_text(json.dumps(self.desktop))
        (self.old/'laptop/receipt.json').write_text(json.dumps(self.row))
        self.marker=dict(session=self.row['session'],packet=self.row['packet'],out=str(self.old))
        self.raw=json.dumps(self.marker).encode();(self.root/'pending.json').write_bytes(self.raw)
        self.digest=hashlib.sha256(self.raw).hexdigest();self.lock=self.root/'common.lock';self.lock.touch()
    def tearDown(self):self.temp.cleanup()
    def test_missing_historical_stop_never_passes_read_only_audit(self):
        with self.assertRaisesRegex(ValueError,'STOP_NOT_CONFIRMED'):recovery.local_proof(self.marker,self.old)
        self.assertEqual((self.root/'pending.json').read_bytes(),self.raw)
    def test_prepare_allows_only_new_stop_check_after_own_groups_absent(self):
        remote=[]
        with patch.object(stop.probe,'absent') as absent,patch.object(stop.probe,'no_robot_processes'):
            digest=stop.prepare(self.root,remote=lambda binding:remote.append(binding),lock_path=self.lock)
        self.assertEqual(digest['marker_sha256'],self.digest);self.assertEqual(len(remote),1);self.assertEqual(absent.call_count,2)
        self.assertEqual((self.root/'pending.json').read_bytes(),self.raw)
    def test_live_forced_or_missing_old_group_refused_before_network(self):
        for field,value in [('forced',True),('group_exit_confirmed',False),('live_pids',[42])]:
            row=copy.deepcopy(self.row);next(iter(row['local']['processes'].values()))[field]=value
            (self.old/'laptop/receipt.json').write_text(json.dumps(row))
            with patch.object(stop.probe,'absent'),patch.object(stop.probe,'no_robot_processes'):
                with self.assertRaises(ValueError):stop.prepare(self.root,remote=lambda _:self.fail('network before group proof'),lock_path=self.lock)
    def forced_nav_row(self):
        row=copy.deepcopy(self.row)
        row['profile']='R5_AUTOMATIC'
        local=row['local']
        for index,name in enumerate(('rviz','yolo','ir_monitor','mission'),start=90020):
            local['processes'][name]=group(index)
        for name in ('startup_health','operator'):
            local['processes'][name]['error']=None
            local['processes'][name]['leader_rc']=2 if name=='operator' else 0
        local.update(requested=True,done=True,stop_confirmed=False,clean=False,group_cleanup_clean=False,
            errors=['OPERATOR_STOP_UNCONFIRMED','PROCESS_UNCLEAN:operator','PROCESS_UNCLEAN:nav','CHILD_LOG_AUDIT_FAILED'])
        row['local']['processes']['nav']=dict(pid=90010,done=True,phase='GROUP_EXITED',
            group_exit_confirmed=True,forced=True,leader_rc=-15,error=None,live_pids=[],
            signals=[dict(signal='SIGINT',scope='LEADER'),dict(signal='SIGTERM',scope='GROUP')])
        label='fire_robot_component_container_isolated-1';child=90011
        row['local']['child_log_audit']=dict(complete=True,clean=False,roles={'nav':dict(
            clean=False,started=1,terminal=0,missing=[dict(label=label,pid=child)],
            errors=['CHILD_SIGNAL_ESCALATION:'+label+':SIGINT_TO_SIGTERM'],internal_signal_fallback=True,
            fallbacks=[dict(label=label,pid=child,timeout_s=5.0,from_signal='SIGINT',to_signal='SIGTERM')]),
            'yolo':children()['roles']['fixture'],'mission':children()['roles']['fixture']})
        return row
    def test_known_forced_nav_can_prepare_only_after_rechecking_all_recorded_pids(self):
        row=self.forced_nav_row()
        (self.old/'laptop/receipt.json').write_text(json.dumps(row))
        with patch.object(stop.probe,'absent') as absent,patch.object(stop.probe,'no_robot_processes'):
            stop.prepare(self.root,remote=lambda _:dict(ok=True),lock_path=self.lock)
        self.assertEqual(absent.call_count,2)
        self.assertIn(90010,absent.call_args.args[0]);self.assertIn(90011,absent.call_args.args[0])
        self.assertEqual((self.root/'pending.json').read_bytes(),self.raw)
    def test_forced_nav_with_incomplete_evidence_fails_closed(self):
        for mutation in ('extra_forced','live_child','wrong_child','wrong_signal','missing_audit','wrong_rc',
                         'wrong_profile','missing_group','missing_role','extra_error'):
            row=self.forced_nav_row();local=row['local'];nav=local['processes']['nav']
            if mutation=='extra_forced':next(v for k,v in local['processes'].items() if k!='nav')['forced']=True
            elif mutation=='live_child':nav['live_pids']=[90011]
            elif mutation=='wrong_child':local['child_log_audit']['roles']['nav']['missing'][0]['pid']=99999
            elif mutation=='wrong_signal':nav['signals'][-1]['signal']='SIGKILL'
            elif mutation=='missing_audit':local['child_log_audit']['complete']=False
            elif mutation=='wrong_rc':nav['leader_rc']=0
            elif mutation=='wrong_profile':row['profile']='DESKTOP_MAPPING'
            elif mutation=='missing_group':del local['processes']['operator']
            elif mutation=='missing_role':del local['child_log_audit']['roles']['mission']
            elif mutation=='extra_error':local['errors'].append('UNEXPECTED_OTHER_FAILURE')
            (self.old/'laptop/receipt.json').write_text(json.dumps(row))
            with self.subTest(mutation=mutation),patch.object(stop.probe,'absent'),patch.object(stop.probe,'no_robot_processes'):
                with self.assertRaises(ValueError):stop.prepare(self.root,remote=lambda _:self.fail('unsafe remote'),lock_path=self.lock)
    def test_forced_nav_live_or_reused_child_rejects_before_network(self):
        (self.old/'laptop/receipt.json').write_text(json.dumps(self.forced_nav_row()))
        with patch.object(stop.probe,'absent',side_effect=ValueError('RECOVERY_PROCESS_STILL_LIVE')):
            with self.assertRaisesRegex(ValueError,'PROCESS_STILL_LIVE'):
                stop.prepare(self.root,remote=lambda _:self.fail('unsafe remote'),lock_path=self.lock)
        self.assertEqual((self.root/'pending.json').read_bytes(),self.raw)

    def test_failed_pulse_shutdown_requires_exact_audit_and_absent_child(self):
        from operational_logs import audit_launch_children
        row = self.forced_nav_row()
        lines = [f'[INFO] [child-{i}]: process started with pid [{91000+i}]\n' for i in range(1, 9)]
        lines += ['[INFO] [fire_pulse_controller-8]: process started with pid [91100]\n',
                  "[INFO] [fire_pulse_controller-8]: sending signal 'SIGINT' to process[fire_pulse_controller-8]\n",
                  '[fire_pulse_controller-8] ACTOR_DRAIN {"reason": "ACTUATOR_OR_MOTION_STOP_UNCONFIRMED"}\n']
        lines += [f'[INFO] [child-{i}]: process has finished cleanly [pid {91000+i}]\n' for i in range(1, 9)]
        lines += ["[ERROR] [fire_pulse_controller-8]: process has died [pid 91100, exit code 2, cmd 'node'].\n"]
        log = ''.join(lines)
        row['local']['child_log_audit']['roles']['mission'] = audit_launch_children(log)
        (self.old/'laptop/mission.log').write_text(log)
        (self.old/'laptop/receipt.json').write_text(json.dumps(row))
        with patch.object(stop.probe,'absent') as absent,patch.object(stop.probe,'no_robot_processes'):
            stop.prepare(self.root,remote=lambda _:dict(ok=True),lock_path=self.lock)
        self.assertIn(91100, absent.call_args.args[0])
        self.assertEqual((self.root/'pending.json').read_bytes(), self.raw)
        # A changed log or missing signal cannot authorize a new attempt.
        (self.old/'laptop/mission.log').write_text(log.replace("sending signal 'SIGINT'", "sending signal 'SIGTERM'"))
        with patch.object(stop.probe,'absent'),patch.object(stop.probe,'no_robot_processes'):
            with self.assertRaisesRegex(ValueError,'MISSION_LOG_INVALID'):
                stop.prepare(self.root,remote=lambda _:self.fail('unsafe remote'),lock_path=self.lock)
    def test_forced_nav_new_stop_still_requires_verified_stop_and_retains_old_failure(self):
        (self.old/'laptop/receipt.json').write_text(json.dumps(self.forced_nav_row()))
        old=(self.old/'laptop/receipt.json').read_bytes();folder=self.new_proof(False)
        with self.assertRaisesRegex(ValueError,'NEW_STOP_NOT_CONFIRMED'):stop.audit_new_stop(self.root,folder,lock_path=self.lock)
        self.assertTrue((self.root/'pending.json').exists())
        # Build a fresh positive STOP-only receipt without changing the old receipt.
        folder2=self.root/('session_'+'3'*32);folder.rename(folder2)
        folder=self.new_proof(True)
        with patch.object(stop.probe,'absent'),patch.object(stop.probe,'no_robot_processes'):
            result=stop.audit_new_stop(self.root,folder,remote=lambda _:dict(ok=True),
                idle_remote=lambda binding,expected_boot:dict(current_boot_id=expected_boot),lock_path=self.lock)
            self.assertIn(90010,result['recheck_pids']);self.assertIn(90011,result['recheck_pids'])
            recovery.archive_verified(self.root,result,'4'*32,lock_path=self.lock)
        self.assertEqual((self.old/'laptop/receipt.json').read_bytes(),old)
    def new_proof(self,stopped=True):
        folder=self.root/('session_'+'2'*32);(folder/'laptop').mkdir(parents=True)
        sid=str(uuid.uuid4());packet='a'*64
        binding=dict(self.row['remote']['binding'],packet=packet,session=str(uuid.uuid4()),boot_id=str(uuid.uuid4()))
        remote=clean_remote(binding)
        row=dict(session=sid,packet=packet,rc=0,profile='DESKTOP_STOP_RECOVERY',local=clean_local(),remote=remote)
        row['local']['stop_confirmed']=stopped
        (folder/'laptop/receipt.json').write_text(json.dumps(row))
        (folder/'desktop_receipt.json').write_text(json.dumps(dict(session=sid,packet=packet,
             robot_stop_confirmed=stopped,cleanup_clean=True,recovery_marker_sha256=self.digest,
             recovery_boot_id=binding['boot_id'])))
        return folder
    def test_new_proof_archives_link_without_rewriting_old_failure(self):
        folder=self.new_proof()
        old=(self.old/'laptop/receipt.json').read_bytes()
        with patch.object(stop.probe,'absent'),patch.object(stop.probe,'no_robot_processes'):
            result=stop.audit_new_stop(self.root,folder,remote=lambda _:dict(ok=True),
                idle_remote=lambda binding,expected_boot:dict(current_boot_id=expected_boot),lock_path=self.lock)
        with patch.object(stop.probe,'absent'),patch.object(stop.probe,'no_robot_processes'):
            recovery.archive_verified(self.root,result,'3'*32,lock_path=self.lock)
        self.assertFalse((self.root/'pending.json').exists())
        saved=json.loads((self.root/'recoveries'/('3'*32)/'audit.json').read_text())
        self.assertIn('CURRENT_STOP_VERIFIED',saved['scope'])
        self.assertNotEqual(saved['session'],saved['new_stop_proof']['session'])
        self.assertEqual((self.old/'laptop/receipt.json').read_bytes(),old)
    def test_new_unconfirmed_stop_or_changed_marker_cannot_archive(self):
        folder=self.new_proof(False)
        with self.assertRaisesRegex(ValueError,'NEW_STOP_NOT_CONFIRMED'):stop.audit_new_stop(self.root,folder,lock_path=self.lock)
        self.assertEqual((self.root/'pending.json').read_bytes(),self.raw)
    def test_verify_action_starts_one_explicit_worker_keeps_pending_and_cannot_scan(self):
        for name in ('maps','runtime'):(self.root/name).mkdir()
        class Fake:
            def __init__(obj,**kw):
                obj.session=str(uuid.uuid4());obj.kind=kw['kind'];obj.out=kw['out'];obj.emit=kw['emit'];obj.started=False
            def start(obj):obj.started=True
            def send(obj,*_):pass
            def heartbeat(obj,*_):pass
        c=SessionController(candidate=self.root,packet='a'*64,state_dir=self.root,runtime=self.root/'runtime',
            map_root=self.root/'maps',map_roots=[self.root/'maps'],session_factory=Fake)
        c.action('VERIFY_STOP');first=c.session
        self.assertTrue(first.started);self.assertEqual(first.kind,'verify_stop')
        c.action('VERIFY_STOP');c.action('SCAN');self.assertIs(c.session,first)
        self.assertEqual((self.root/'pending.json').read_bytes(),self.raw)
        first.emit(dict(event='FINISHED',session=first.session,robot_stop_confirmed=False,
                        cleanup_clean=False,session_was_started=True))
        c.tick();self.assertIsNone(c.session);self.assertEqual(c.phase,'BLOCKED')
        self.assertEqual((self.root/'pending.json').read_bytes(),self.raw)
