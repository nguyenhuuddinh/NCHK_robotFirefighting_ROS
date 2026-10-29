"""Owned Pi demo lifecycle over SSH; no ROS dependency, no persistent config.

The same source runs on Pi via python -c. EOF, heartbeat expiry, child exit or
STOP all clean up its descendants, including detached ROS launch children.
"""
import ctypes
import fcntl
import getpass
import json
import math
import os
from pathlib import Path
import queue
import select
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time


LEASE_SECONDS = 35.0
CLOCK_BOUND_SECONDS = 0.1


def process_table():
    rows = {}
    for p in Path('/proc').glob('[0-9]*'):
        try:
            f = (p / 'stat').read_text().rsplit(')', 1)[1].split()
            rows[int(p.name)] = (int(f[1]), f[19], f[0])
        except (OSError, ValueError, IndexError):
            continue  # Process exited during snapshot.
    return rows


def descendants(exclude_roots=()):
    rows = process_table()
    owned = {os.getpid()}
    while True:
        new = {pid for pid, (parent, _, _) in rows.items()
               if parent in owned} - owned
        if not new:
            break
        owned.update(new)
    excluded = set(exclude_roots)
    while True:
        new = {pid for pid, (parent, _, _) in rows.items()
               if parent in excluded} - excluded
        if not new:
            break
        excluded.update(new)
    return {pid: rows[pid] for pid in owned - excluded if pid != os.getpid()}


def install_subreaper():
    if ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), 'Cannot install child subreaper')


def cleanup_descendants(stages=((signal.SIGINT, 5), (signal.SIGTERM, 3),
                                (signal.SIGKILL, 2)), exclude_roots=()):
    """Only this Python process's children; PID birth checked before signal."""
    for sig, seconds in stages:
        end = time.monotonic() + seconds
        sent = set()
        while time.monotonic() < end:
            while True:
                try:
                    if os.waitpid(-1, os.WNOHANG)[0] == 0:
                        break
                except ChildProcessError:
                    break
            owned = descendants(exclude_roots)
            if not owned:
                return []
            current = process_table()
            for pid, (_, birth, state) in owned.items():
                if state == 'Z' or (pid, birth) in sent:
                    continue
                if current.get(pid, (None, None))[1] != birth:
                    continue
                try:
                    os.kill(pid, sig)
                    sent.add((pid, birth))
                except ProcessLookupError:
                    continue
            time.sleep(.05)
    return sorted(descendants(exclude_roots))


def require_idle(proc_root=Path('/proc')):
    """Reject old/manual stacks instead of stepping their clock or claiming them."""
    terms = {'serial_bridge_node', 'micro_ros_agent', 'native_mjpeg_camera',
             'camsense_x1_node', 'safety_watchdog', 'odom_to_tf_broadcaster',
             'usb_cam_node_exe', 'v4l2_camera_node', 'scan_qos_relay',
             'camera_qos_relay', 'robot_state_publisher', 'rosbridge_websocket',
             'rosapi_node'}
    found = []
    for p in proc_root.glob('[0-9]*'):
        try:
            args = (p / 'cmdline').read_bytes().split(b'\0')
            names = {os.path.basename(a).decode(errors='replace') for a in args if a}
            if names & terms or ('ros2' in names and b'launch' in args):
                found.append(int(p.name))
        except OSError:
            continue
    if found:
        raise RuntimeError('Pi còn launch/node cũ PID=' + str(found) +
                           '. Dừng phiên cũ trên Pi trước; chưa chỉnh giờ hoặc mở launch.')


def pi_serve(launch_command=None, lease=LEASE_SECONDS, check_idle=require_idle,
             lock_path=None):
    """JSON line RPC on SSH stdin/stdout; dummy argv may be injected offline."""
    lock_path = lock_path or f'/tmp/fire_robot_demo_{os.getuid()}.lock'
    lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    install_subreaper()
    out = Path(tempfile.mkdtemp(prefix='fire_robot_pi_session_'))
    child = None
    stop = [False]
    request_id = None
    result = {'reason': 'setup', 'remaining_pids': None, 'log_dir': str(out)}
    def request_stop(*_):
        stop[0] = True
    def reply(data):
        print(json.dumps({**data, 'request_id': request_id}), flush=True)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_stop)
    buffer = b''
    last_contact = time.monotonic()
    try:
        while not stop[0]:
            if child is not None and child.poll() is not None:
                result['reason'] = 'launch_exited'
                break
            if child is not None and time.monotonic() - last_contact > lease:
                result['reason'] = 'controller_timeout'
                break
            if b'\n' not in buffer:
                ready, _, _ = select.select([sys.stdin.fileno()], [], [], .2)
                if not ready:
                    continue
                data = os.read(sys.stdin.fileno(), 8192)
                if not data:
                    result['reason'] = 'controller_eof'
                    break
                buffer += data
                if len(buffer) > 32768:
                    raise RuntimeError('Control message too large')
                if b'\n' not in buffer:
                    continue
            raw, buffer = buffer.split(b'\n', 1)
            req = json.loads(raw)
            op = req.get('op')
            request_id = req.get('request_id')
            last_contact = time.monotonic()
            if op == 'stop':
                result['reason'] = 'stop_requested'
                break
            if op == 'ping':
                reply({'ok': True, 'running': child is not None})
                continue
            if child is not None:
                raise RuntimeError('Không chỉnh giờ/start lần hai khi Pi launch đang chạy')
            check_idle()
            if op == 'clock':
                reply({'ok': True, 'time': time.time(), 'log_dir': str(out)})
            elif op == 'authorize':
                password = req.pop('password', None)
                cmd = ['sudo', '-n', '-v'] if password is None else ['sudo', '-S', '-p', '', '-v']
                proc = subprocess.run(cmd, input='' if password is None else password+'\n',
                                      text=True, capture_output=True, timeout=12)
                reply({'ok': True, 'authorized': proc.returncode == 0})
            elif op == 'set_clock':
                epoch = float(req['epoch'])
                if not math.isfinite(epoch) or epoch < 1577836800:
                    raise RuntimeError('Invalid laptop timestamp')
                proc = subprocess.run(['sudo', '-n', 'date', '-u',
                                       '--set=@'+format(epoch, '.6f')],
                                      capture_output=True, text=True, timeout=5)
                if proc.returncode:
                    raise RuntimeError('Không chỉnh được giờ Pi bằng sudo date; kiểm tra quyền sudo')
                reply({'ok': True})
            elif op == 'start':
                workspace = Path.home() / 'NCHK_robotFirefighting_ROS/ros2_ws'
                command = launch_command or ['bash', '-c',
                    'cd '+shlex.quote(str(workspace))+' && '
                    'source /opt/ros/humble/setup.bash && source install/setup.bash && '
                    'export ROS_DOMAIN_ID=0 && unset ROS_LOCALHOST_ONLY && '
                    'exec ros2 launch fire_robot_bringup robot.launch.py '
                    'use_sim_time:=false use_camera:=true camera_backend:=native_mjpeg '
                    'camera_fps:=15 camera_max_bytes_per_sec:=750000']
                with (out / 'robot.log').open('w') as log:
                    child = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                                             stdout=log, stderr=subprocess.STDOUT,
                                             start_new_session=True)
                result['launch_pid'] = child.pid
                last_contact = time.monotonic()
                reply({'ok': True, 'pid': child.pid, 'log_dir': str(out)})
            else:
                raise RuntimeError('Unknown control operation')
    except BaseException as exc:
        # Never include the input request (it can contain a sudo password).
        result['reason'] = 'error'
        result['error'] = str(exc)
    finally:
        if stop[0]:
            result['reason'] = 'signal'
        result['remaining_pids'] = cleanup_descendants()
        result['ok'] = not result['remaining_pids'] and 'error' not in result
        result['stopped'] = True
        result['request_id'] = request_id
        (out / 'result.json').write_text(json.dumps(result, indent=2))
        try:
            reply(result)
        except BrokenPipeError:
            pass  # Result persists on Pi even when SSH has disconnected.
        os.close(lock_fd)


class PiSession:
    """Laptop controller; all I/O and shutdown have explicit deadlines."""
    def __init__(self, host, log_dir, event):
        self.event = event
        self.host = host
        self.lock = threading.Lock()
        self.responses = queue.Queue()
        self.end = threading.Event()
        self.failure = None
        self.process = None
        self.worker = None
        self.reader = None
        self.remote_log = None
        self.sequence = 0
        self.terminal_result = None
        self.log_dir = Path(log_dir)

    def connect(self):
        source = Path(__file__).read_text()
        command = shlex.join(['python3', '-u', '-c', source])
        with (self.log_dir / 'pi_ssh.log').open('w') as log:
            self.process = subprocess.Popen(
                ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                 '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2',
                 self.host, command], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=log, text=True, start_new_session=True)
        def read():
            try:
                for line in self.process.stdout:
                    self.responses.put(json.loads(line))
            except (OSError, ValueError) as exc:
                self.responses.put({'ok': False, 'error': 'Invalid SSH response: '+str(exc)})
            finally:
                self.responses.put({'ok': False, 'error': 'SSH Pi disconnected'})
        self.reader = threading.Thread(target=read, daemon=True)
        self.reader.start()

    def request(self, op, timeout=15, **data):
        with self.lock:
            if self.failure:
                raise RuntimeError(self.failure)
            try:
                self.sequence += 1
                self.process.stdin.write(json.dumps(
                    {'op': op, 'request_id': self.sequence, **data})+'\n')
                self.process.stdin.flush()
                deadline = time.monotonic() + timeout
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise queue.Empty()
                    response = self.responses.get(timeout=remaining)
                    if (response.get('request_id') == self.sequence or
                            response.get('stopped') or not response.get('ok')):
                        break  # Discard late replies from an interrupted request.
                if response.get('log_dir'):
                    self.remote_log = response['log_dir']
                if response.get('stopped'):
                    self.terminal_result = response
                if not response.get('ok') or (response.get('stopped') and op != 'stop'):
                    raise RuntimeError(response.get('error') or 'Pi supervisor stopped: '+str(response))
                return response
            except (OSError, queue.Empty, RuntimeError) as exc:
                self.failure = 'Pi control failed: '+(str(exc) or 'response timeout')
                raise RuntimeError(self.failure) from exc

    def measure_clock(self):
        samples = []
        for _ in range(3):
            start, mono = time.time(), time.monotonic()
            response = self.request('clock')
            end = time.time()
            rtt = time.monotonic() - mono
            if abs((end-start)-rtt) > .05:
                raise RuntimeError('Giờ laptop thay đổi trong lúc đo; chạy lại sau khi giờ ổn định')
            self.remote_log = response['log_dir']
            samples.append((rtt, response['time']-(start+end)/2))
        rtt, offset = min(samples)
        if rtt > CLOCK_BOUND_SECONDS:
            raise RuntimeError(f'SSH RTT={rtt:.3f}s quá cao để xác minh giờ; chưa mở launch')
        return offset, rtt

    def synchronize(self):
        offset, rtt = self.measure_clock()
        self.event('PI_CLOCK_BEFORE', offset_s=offset, rtt_s=rtt)
        if abs(offset)+rtt/2 > CLOCK_BOUND_SECONDS:
            if not self.request('authorize')['authorized']:
                if not sys.stdin.isatty():
                    raise RuntimeError('Cần terminal để nhập mật khẩu sudo Pi và đồng bộ giờ')
                password = getpass.getpass('Mật khẩu sudo Pi (không lưu): ')
                try:
                    authorized = self.request('authorize', password=password)['authorized']
                finally:
                    del password
                if not authorized:
                    raise RuntimeError('Sudo Pi không được cấp quyền; chưa mở launch')
            # Fresh timestamp after the human password prompt, never before it.
            self.request('set_clock', epoch=time.time()+rtt/2)
            offset, rtt = self.measure_clock()
            if abs(offset)+rtt/2 > CLOCK_BOUND_SECONDS:
                raise RuntimeError(f'Giờ Pi còn lệch {offset:.3f}s; chưa mở launch')
        self.event('PI_CLOCK_VERIFIED', offset_s=offset, rtt_s=rtt,
                   max_error_s=abs(offset)+rtt/2)
        print('[OK] Giờ Pi khớp laptop trong ngưỡng 100 ms.', flush=True)

    def start(self):
        if os.environ.get('FIRE_AUTO_START_PI', '1') != '1':
            raise RuntimeError('Demo cần sở hữu Pi launch để cleanup; FIRE_AUTO_START_PI phải là 1')
        self.connect()
        self.synchronize()
        response = self.request('start')
        self.remote_log = response['log_dir']
        self.event('PI_SESSION_STARTED', **response)
        def heartbeat():
            while not self.end.wait(3):
                try:
                    self.request('ping', timeout=8)
                except RuntimeError:
                    return  # Main session notices failure and withdraws arms.
        self.worker = threading.Thread(target=heartbeat, daemon=True)
        self.worker.start()

    def assert_alive(self):
        if self.failure or (self.process and self.process.poll() is not None):
            raise RuntimeError(self.failure or 'SSH Pi đã thoát')

    def close(self):
        self.end.set()
        if self.worker:
            self.worker.join(timeout=9)
        if self.process is None:
            return True
        result = self.terminal_result
        error = None
        try:
            if result is None:
                result = self.request('stop', timeout=15)
        except RuntimeError as exc:
            error = str(exc)
        finally:
            try:
                self.process.stdin.close()  # EOF also requests remote cleanup.
            except OSError:
                pass
            try:
                self.process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
            if self.reader:
                self.reader.join(timeout=1)
        # A timed-out/interrupted RPC may still be followed by a valid final
        # cleanup receipt after EOF. Require that receipt, not SSH exit alone.
        while True:
            try:
                response = self.responses.get_nowait()
            except queue.Empty:
                break
            if response.get('stopped'):
                result = response
        confirmed = bool(result and result.get('stopped') is True and
                         result.get('remaining_pids') == [])
        if confirmed:
            self.event('PI_SESSION_STOPPED', **result)
        else:
            self.event('PI_CLEANUP_UNCONFIRMED', error=error,
                       result=result, log_dir=self.remote_log,
                       lease_seconds=LEASE_SECONDS)
        return confirmed


if __name__ == '__main__':
    pi_serve()
