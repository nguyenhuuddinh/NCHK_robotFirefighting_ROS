"""Remote shell construction preserves ROS imports and never runs SSH here."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from services.live_session import LiveSession

class LaunchEnvironmentTests(unittest.TestCase):
    def test_pi_export_preserves_sourced_ros_pythonpath_and_script_has_valid_syntax(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);capture=[]
            (root/'ros_marker.py').write_text('VALUE=77\n')
            runner=LiveSession(candidate=root,packet='a'*64,runtime=root,out=root,
                map_root=root,map_roots=[root],kind='fire',selected_map=None,emit=lambda _:None)
            with patch('services.live_session.PtyChild',side_effect=lambda args:capture.append(args)),patch.object(runner,'_owned_pi'):
                runner._launch_pi()
            script=shlex.split(capture[0][-1])[-1]
            syntax=subprocess.run(['bash','-n'],input=script,text=True,capture_output=True)
            self.assertEqual(syntax.returncode,0,syntax.stderr)
            self.assertIn('/desktop/generated/app_pi_entry.py',script)
            self.assertNotIn('/r5_staging/',script)
            self.assertIn('PYTHONDONTWRITEBYTECODE=1',script)
            export=next(x for x in script.splitlines() if x.startswith('export PYTHONPATH='))
            # Execute only the export statement in a local fake import test.
            command=export+'\n/usr/bin/python3 -B -c "import ros_marker; assert ros_marker.VALUE == 77"'
            result=subprocess.run(['bash','-c',command],env=dict(os.environ,PYTHONPATH=str(root)),capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)

    def test_environment_record_is_an_allowlist_without_private_values(self):
        from services.app_environment import record,KEYS
        from services.live_session import atomic
        import json
        with tempfile.TemporaryDirectory() as tmp,patch.dict(os.environ,{'PRIVATE_PASSWORD':'not-for-log'}):
            root=Path(tmp);record(root,root,'laptop',atomic)
            raw=(root/'app_environment.json').read_text()
            self.assertNotIn('PRIVATE_PASSWORD',raw);self.assertNotIn('not-for-log',raw)
            self.assertEqual(set(json.loads(raw)['environment']),set(KEYS))
