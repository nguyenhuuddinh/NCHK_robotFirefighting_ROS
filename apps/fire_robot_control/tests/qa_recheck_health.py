"""Actual health entry with no injected delay, localhost213 only."""
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
def test_actual_app_health_entry_normal_load(mode):
    output=Path(os.environ.get('FIRE_ROBOT_QA_HEALTH_DIR','/tmp/fire_app_qa2_20261030'))/('normal_health_'+mode)
    assert not output.exists(),'Preserve existing traces; use a fresh evidence directory'
    real_popen=subprocess.Popen
    def start(argv,**kw):
        if '--role' in argv and argv[argv.index('--role')+1]=='health':
            argv=[sys.executable,'-B',str(APP/'tests/app_health_normal_worker.py'),
                  '--source',str(ROOT/'releases/r5_20261029/ros'),'--output',str(output),'--mode',mode]
        return real_popen(argv,**kw)
    with patch.object(subprocess,'Popen',start):
        result=run_probe.experiment(ROOT/'releases/r5_20261029/ros',output,7.,'callback','none',0.)
    metrics=result['metrics']
    assert metrics['fast_callback_source_age_ms']['p95']<300,metrics
    assert metrics['health_lookup_source_age_ms']['p95']<300,metrics
    assert all(count==0 for count in result['trace_dropped'].values())
    (output/'app_entry_result.json').write_text(json.dumps(dict(mode=mode,metrics=metrics,
        fake_sample_delay_ms=0,scope='LOCALHOST_NORMAL_SYNTHETIC_NOT_ROBOT_ACCEPTANCE'),indent=2)+'\n')
