"""Exercise a built app's real operator and helper only on ROS localhost 211.

There is deliberately no robot telemetry. Q must finish with an unconfirmed
STOP, rc2 and no surviving owned process; it must never manufacture STOP proof.
"""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

from services.operator_bootstrap import create_bootstrap_file
from services.operator_launch import launch_candidate_operator
from services.operator_runner import OperatorBootstrap
from services.package_candidate import verify_artifact
from operational_owned_process import Backend


def main():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID') == '211'
    artifact, output = Path(sys.argv[1]), Path(sys.argv[2])
    receipt = verify_artifact(artifact)
    fixture = Path(f'/tmp/fire_robot_control_preview_{os.getuid()}/fire_robot_frontend_fixture')
    with tempfile.TemporaryDirectory(prefix='app-operator-smoke-') as tmp:
        root = Path(tmp)
        runtime, session = root/'runtime', root/'session'
        runtime.mkdir(mode=0o700)
        session.mkdir(mode=0o700)
        boot = OperatorBootstrap('8518a120-a012-4db3-bbf3-f73f67aed61e', 'b'*32, 'c'*32,
                                 receipt['packet'], runtime/'ui.sock')
        ui = None
        with (root/'operator.log').open('wb') as log:
            operator = launch_candidate_operator(boot, app_dir=artifact/'ros/desktop',
                candidate_root=artifact/'ros', session_dir=session, runtime_dir=runtime,
                env=dict(os.environ), stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic()+6.
                while not boot.socket_path.exists() and operator.poll() is None and time.monotonic()<deadline:
                    time.sleep(.01)
                assert boot.socket_path.exists(), 'Operator did not open helper socket'
                with create_bootstrap_file(boot, runtime) as private:
                    ui = subprocess.Popen([str(fixture), str(boot.socket_path), 'actions', str(private.fileno())],
                        pass_fds=(private.fileno(),), stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
                rc = operator.wait(timeout=9.)
            finally:
                if operator.poll() is None:
                    operator.send_signal(signal.SIGINT)
                    operator.wait(timeout=8.)
                if ui is not None:
                    if ui.poll() is None:
                        ui.terminate()
                    ui.communicate(timeout=2.)
        text = (root/'operator.log').read_text()
        shutil.copy2(root/'operator.log', output/'packaged_operator.log')
        diagnostic = json.loads((session/'operator_ui.json').read_text())
        shutil.copy2(session/'operator_ui.json', output/'packaged_operator_ui.json')
        terminal = json.loads((session/'operator_result.json').read_text())
        shutil.copy2(session/'operator_result.json', output/'packaged_operator_terminal.json')
        stops = [json.loads(line.split('OPERATOR_STOP ',1)[1]) for line in text.splitlines() if 'OPERATOR_STOP {' in line]
        assert rc == 2 and len(stops) == 1 and stops[0]['done'] and stops[0]['confirmed'] is False, text
        assert stops[0]['cause'] == 'Q' and diagnostic['reason'] == 'OPERATOR_Q', text
        assert terminal['scope'] == 'APP_OPERATOR_TERMINAL_RESULT_NOT_STOP_PROOF'
        assert (terminal['session'], terminal['packet'], terminal['pid']) == (boot.session, boot.packet, operator.pid)
        assert terminal['runtime_status'] == 'FAILED' and terminal['process_exit_code'] == 2
        assert terminal['stop_confirmed'] is False and terminal['process_cleanup_ok'] is False
        assert not boot.socket_path.exists()
        live = [p for p in Backend().snapshot(operator.pid) if p['state'] not in ('Z', 'X')]
        assert live == [], live
        result = dict(packet=receipt['packet'], scope='LOCALHOST_NO_ROBOT', rc=rc,
            no_telemetry_correctly_unconfirmed=True, ui_reason=diagnostic['reason'],
            owned_processes_absent=True, socket_removed=True, terminal_result_correctly_failed=True)
        (output/'packaged_operator_result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result))


if __name__ == '__main__':
    main()
