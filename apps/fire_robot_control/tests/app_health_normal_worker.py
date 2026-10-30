"""Normal-load isolated instrumentation of the actual app health entry; no source edits."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
p=argparse.ArgumentParser();p.add_argument('--source',type=Path);p.add_argument('--output',type=Path);p.add_argument('--mode',choices=('r5','sampled'));a=p.parse_args()
diag=Path(__file__).resolve().parents[1]/'diagnostics/tf_startup';sys.path.insert(0,str(diag))
from common import Trace,require_isolation,verify_source,source_age_ms,stamp_ns
require_isolation();verify_source(a.source)
for package in ('fire_robot_mission','fire_robot_bringup'):sys.path.insert(0,str(a.source/'ros2_ws/src'/package))
from fire_robot_mission import startup_health as original
from services.app_health_entry import main
assert Path(original.__file__).resolve()==(a.source/'ros2_ws/src/fire_robot_mission/fire_robot_mission/startup_health.py').resolve()
trace=Trace();Base=original.StartupHealthNode
class Instrumented(Base):
    def __init__(self,*args):
        super().__init__(*args)
        receive=self.tf.set_transform
        def observed(transform,authority):
            if transform.header.frame_id=='odom' and transform.child_frame_id=='base_link':
                trace.add('tf_callback',stamp_ns=stamp_ns(transform),source_age_ms=source_age_ms(stamp_ns(transform),time.time_ns()))
            return receive(transform,authority)
        self.tf.set_transform=observed
        (a.output/'health.ready').write_text(json.dumps({'scope':'SYNTHETIC_ONLY','mode':a.mode})+'\n')
    def sample(self):
        before=time.monotonic_ns();row=super().sample();time.sleep(0.)
        trace.add('sample',sample_cost_ms=(time.monotonic_ns()-before)/1e6,write_cost_ms=None,
            tf_source_age_ms=row['io_diagnostics']['detail']['odom_tf'].get('source_age_ms'),
            io=row['checks']['io'],blockers=row['io_diagnostics']['blockers'])
        return row
original.StartupHealthNode=Instrumented
# Only this worker is signalled if the diagnostic parent disappears.
signal.signal(signal.SIGALRM,lambda *_:os.kill(os.getpid(),signal.SIGTERM));signal.alarm(25)
try:main(['--out',str(a.output/'observer_health.json'),'--session','diagnostic-fixture','--role','pi','--mode',a.mode])
finally:
    signal.alarm(0);trace.save(a.output,'health');verify_source(a.source)
