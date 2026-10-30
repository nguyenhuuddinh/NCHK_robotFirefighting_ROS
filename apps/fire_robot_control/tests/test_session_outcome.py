"""Real saved startup failure replay, with negative safety-proof mutations."""
import ast
import copy
import json
from pathlib import Path
import tempfile
import unittest
from services.session_outcome import outcome,primary_error
from services.live_session import receipt_result
from proof_fixture import clean_local,clean_remote
APP=Path(__file__).resolve().parents[1]
EVIDENCE=APP/'docs/qa_evidence/startup_block_215926_220156_20261029/raw_laptop'

class OutcomeTests(unittest.TestCase):
    def test_real_fire_rc2_clean_stop_before_fail_after_pass(self):
        row=json.loads((EVIDENCE/'fire/laptop/receipt.json').read_text())
        module=ast.parse((APP/'tests/fixtures/receipt_result_6734.py').read_text())
        func=next(x for x in module.body if isinstance(x,ast.FunctionDef) and x.name=='receipt_result')
        scope={};exec(compile(ast.Module(body=[func],type_ignores=[]),'old_receipt','exec'),scope)
        self.assertEqual(scope['receipt_result'](row,row['session'],row['packet']),(True,False))
        self.assertEqual(receipt_result(row,row['session'],row['packet']),(True,True))
        self.assertEqual(row['rc'],2)
    def test_real_mapping_missing_stop_still_blocks(self):
        row=json.loads((EVIDENCE/'mapping/laptop/receipt.json').read_text())
        self.assertEqual(receipt_result(row,row['session'],row['packet']),(False,False))
    def test_rc2_alone_is_neither_stop_nor_cleanup_proof(self):
        row=dict(session='s',packet='p',rc=2,local=dict(clean=True),remote=dict(confirmed=True))
        self.assertEqual(outcome(row,'s','p'),(False,False))
    def test_remote_startup_error_only_is_distinct_from_child_or_group_fault(self):
        row=dict(session='s',packet='p',rc=2,local=clean_local(),remote=clean_remote({'packet':'p'}))
        row['remote'].update(confirmed=False,cleanup_confirmed=True,runner_error='PI_SEAL_HEALTH_TIMEOUT')
        self.assertEqual(outcome(row,'s','p'),(True,True))
        for scope,field,value in [('local','forced',True),('remote','leader_rc',-15),('local','live_pids',[123]),('remote','group_exit_confirmed',False)]:
            bad=copy.deepcopy(row)
            groups=bad['local']['processes'] if scope=='local' else bad['remote']['receipt']['groups']
            next(iter(groups.values()))[field]=value
            self.assertEqual(outcome(bad,'s','p'),(True,False))
    def test_missing_audit_or_wrong_binding_refused(self):
        row=dict(session='s',packet='p',rc=0,local=clean_local(),remote=clean_remote({'packet':'p'}))
        self.assertEqual(outcome(row,'s','p',remote_binding={'packet':'wrong'}),(False,False))
        for field in ('child_log_audit','processes','group_cleanup_clean'):
            bad=copy.deepcopy(row);bad['local'].pop(field)
            self.assertEqual(outcome(bad,'s','p'),(True,False))
    def test_primary_error_preserved_and_unknown_ssh_not_read(self):
        self.assertIn('SEAL_PI',primary_error(EVIDENCE/'fire/laptop','PI_RUNNER_EXITED_DURING_SESSION'))
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'laptop_runner.log').write_text('private raw ssh')
            self.assertEqual(primary_error(p,'fallback'),'fallback')

    def test_only_bound_allowlisted_ui_failure_replaces_operator_exit(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            error='RuntimeError:OWNED_CHILD_EXIT:operator'
            (folder/'failure.json').write_text(json.dumps({'error':error}))
            (folder/'receipt.json').write_text(json.dumps({'session':'s','packet':'p'}))
            row=dict(schema_version=1,session='s',packet='p',scope='UI_LIVENESS_ONLY_NOT_STOP_PROOF',reason='FRONTEND_LEASE_EXPIRED')
            (folder/'operator_ui.json').write_text(json.dumps(row))
            self.assertEqual(primary_error(folder),'OPERATOR_UI:FRONTEND_LEASE_EXPIRED')
            for mutation in ({'session':'old'}, {'packet':'wrong'}, {'reason':'private content'}, {'scope':'STOP_CONFIRMED'}):
                (folder/'operator_ui.json').write_text(json.dumps(dict(row,**mutation)))
                self.assertEqual(primary_error(folder),error)
