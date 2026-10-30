"""Actual app entry vs controlled load; isolated localhost213, never hardware."""
import json
import os
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch
import pytest
APP=Path(__file__).resolve().parents[1];ROOT=APP.parents[1]
sys.path.insert(0,str(APP/'diagnostics/tf_startup'))
import run_probe

@pytest.mark.parametrize('mode',['r5','sampled'])
def test_actual_health_entry_with_25ms_controlled_load(mode):
    output=Path(os.environ.get('FIRE_ROBOT_QA_HEALTH_DIR','/tmp/fire_app_session1_20261030'))/('actual_health_load_'+mode)
    assert not output.exists(),'Use a fresh diagnostic output; preserve old traces'
    real_popen=subprocess.Popen
    def start(argv,**kw):
        if '--role' in argv and argv[argv.index('--role')+1]=='health':
            argv=[sys.executable,'-B',str(APP/'tests/app_health_load_worker.py'),
                  '--source',str(ROOT/'releases/r5_20261029/ros'),'--output',str(output),'--mode',mode]
        return real_popen(argv,**kw)
    with patch.object(subprocess,'Popen',start):
        result=run_probe.experiment(ROOT/'releases/r5_20261029/ros',output,7.,'callback','none',25.)
    metrics=result['metrics']
    assert metrics['fast_callback_source_age_ms']['p95']<300,metrics
    age=metrics['health_lookup_source_age_ms']['p95']
    if mode=='r5':assert age>1000,metrics
    else:assert age<300,metrics
    assert all(count==0 for count in result['trace_dropped'].values())
    (output/'app_entry_result.json').write_text(json.dumps(dict(mode=mode,metrics=metrics,
        fake_sample_delay_ms=25,scope='LOCALHOST_SYNTHETIC_NOT_ROBOT_ACCEPTANCE'),indent=2)+'\n')
