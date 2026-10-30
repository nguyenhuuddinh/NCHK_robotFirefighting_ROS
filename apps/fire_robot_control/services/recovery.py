"""Explicit, asynchronous retry audit. Never starts or stops a robot."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import threading
import time
import uuid

from . import recovery_probe as probe
from .live_session import atomic


def pending_record(state_dir):
    state_dir = Path(state_dir).resolve(strict=True)
    marker, digest = probe.read_object(state_dir / 'pending.json')
    probe.identity(marker.get('session'), marker.get('packet'))
    out = Path(marker.get('out', ''))
    probe.require(out.parent == state_dir and re.fullmatch('session_[0-9a-f]{32}', out.name)
                  and out.resolve(strict=True) == out and not out.is_symlink(), 'RECOVERY_LOG_PATH_INVALID')
    return marker, digest, out


def local_proof(marker, out):
    desktop, desktop_sha = probe.read_object(out / 'desktop_receipt.json')
    receipt, receipt_sha = probe.read_object(out / 'laptop/receipt.json')
    for row in (desktop, receipt):
        probe.require(row.get('session') == marker['session'] and row.get('packet') == marker['packet'],
                      'RECOVERY_LOCAL_BINDING_MISMATCH')
    probe.require(desktop.get('robot_stop_confirmed') is True and desktop.get('session_was_started') is True,
                  'RECOVERY_STOP_NOT_CONFIRMED')
    # Both desktop workflows use the same STOP, process-group and child
    # receipts. Accept only these known profiles; no cleanup check is waived.
    probe.require(receipt.get('profile') in ('DESKTOP_MAPPING', 'R5_AUTOMATIC'),
                  'RECOVERY_PROFILE_UNSUPPORTED')
    local = receipt.get('local', {})
    probe.require(local.get('stop_confirmed') is True and local.get('clean') is True
                  and local.get('group_cleanup_clean') is True and local.get('errors') == [],
                  'RECOVERY_LOCAL_CLEANUP_MISSING')
    probe.child_clean(local.get('child_log_audit'))
    pids = probe.groups_clean(local.get('processes'))
    remote = receipt.get('remote', {})
    binding = remote.get('binding')
    probe.require(isinstance(binding, dict) and binding.get('packet') == marker['packet']
                  and 'secret' not in binding, 'RECOVERY_REMOTE_BINDING_MISSING')
    probe.remote_cleanup(remote.get('receipt'))
    return pids, binding, dict(desktop_receipt_sha256=desktop_sha, laptop_receipt_sha256=receipt_sha)


def pi_host(run=subprocess.run):
    # Avoid sending SSH to an unrelated router also using 10.0.0.1.
    for host, dev, src in (('10.0.0.1', 'wlp8s0', '10.0.0.253'),
                           ('10.42.0.185', 'enp7s0', '10.42.0.1')):
        result = run(['ip', '-j', 'route', 'get', host], capture_output=True, text=True, timeout=3, check=False)
        if result.returncode:
            continue
        rows = json.loads(result.stdout)
        if (isinstance(rows, list) and len(rows) == 1 and isinstance(rows[0], dict)
                and rows[0].get('dev') == dev and rows[0].get('prefsrc') == src
                and not rows[0].get('gateway')):
            return host
    raise ValueError('RECOVERY_CONNECT_PI_WIFI_OR_LAN')


def remote_proof(binding, run=subprocess.run):
    host = pi_host(run)
    code = Path(probe.__file__).read_text() + '\nremote_main(' + repr(binding) + ')\n'
    result = run(['ssh', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                  '-o', 'ConnectTimeout=8', 'pi@' + host, '/usr/bin/python3 -'],
                 input=code, capture_output=True, text=True, timeout=18, check=False)
    probe.require(result.returncode == 0, 'RECOVERY_PI_SSH_FAILED')
    row = probe.decode(result.stdout)
    probe.require(row.get('ok') is True, 'RECOVERY_PI_CHECK_FAILED:' + str(row.get('error', 'UNKNOWN'))[:100])
    proof = row.get('proof', {})
    probe.require(proof.get('binding') == binding and all(proof.get(k) is True for k in
                  ('processes_absent', 'common_lock_free', 'boot_matches')), 'RECOVERY_PI_PROOF_INVALID')
    probe.remote_cleanup(proof.get('cleanup'))
    probe.require(re.fullmatch('[0-9a-f]{64}', proof.get('done_sha256', '')), 'RECOVERY_PI_HASH_INVALID')
    return proof


def remote_idle_proof(binding, *, expected_boot=None, run=subprocess.run):
    """Read-only precheck for an explicitly requested fresh STOP session."""
    host = pi_host(run)
    nonce = uuid.uuid4().hex
    code = Path(probe.__file__).read_text() + '\nremote_idle_main(' + repr(binding) + ', ' + repr(expected_boot) + ', ' + repr(nonce) + ')\n'
    result = run(['ssh', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                  '-o', 'ConnectTimeout=8', 'pi@' + host, '/usr/bin/python3 -'],
                 input=code, capture_output=True, text=True, timeout=18, check=False)
    probe.require(result.returncode == 0, 'RECOVERY_PI_SSH_FAILED')
    row = probe.decode(result.stdout)
    # Accept only protocol codes, never arbitrary remote contents as UI text.
    if row.get('ok') is not True:
        error = row.get('error')
        if not isinstance(error, str) or not re.fullmatch('RECOVERY_[A-Z0-9_]+', error):
            error = 'RECOVERY_PI_CHECK_FAILED'
        raise ValueError(error)
    proof = row.get('proof')
    probe.require(isinstance(proof, dict), 'RECOVERY_PI_PROOF_INVALID')
    current = proof.get('current_boot_id')
    probe.require(isinstance(current, str) and re.fullmatch('[0-9a-f-]{36}', current)
                  and str(uuid.UUID(current)) == current, 'RECOVERY_PI_BOOT_INVALID')
    probe.require(proof.get('binding') == binding and proof.get('request_nonce') == nonce
                  and all(proof.get(k) is True for k in ('processes_absent', 'common_lock_free', 'boot_stable'))
                  and re.fullmatch('[0-9a-f]{64}', proof.get('pairing_sha256', '')),
                  'RECOVERY_PI_PROOF_INVALID')
    probe.require(expected_boot is None or current == expected_boot, 'RECOVERY_PI_BOOT_CHANGED')
    mode = proof.get('mode')
    allowed = {'SAME_BOOT_CLEANUP', 'SAME_BOOT_QUIESCENT_ONLY'} if current == binding.get('boot_id') else {'NEW_BOOT_QUIESCENT_ONLY'}
    probe.require(mode in allowed, 'RECOVERY_PI_PROOF_INVALID')
    if mode == 'SAME_BOOT_CLEANUP':
        old = proof.get('historical_cleanup', {})
        probe.require(old.get('binding') == binding and old.get('boot_matches') is True
                      and old.get('processes_absent') is True and old.get('common_lock_free') is True,
                      'RECOVERY_PI_PROOF_INVALID')
        probe.remote_cleanup(old.get('cleanup'))
        probe.require(re.fullmatch('[0-9a-f]{64}', old.get('done_sha256', '')), 'RECOVERY_PI_HASH_INVALID')
    elif mode == 'SAME_BOOT_QUIESCENT_ONLY':
        pids = proof.get('pids_checked')
        digest = proof.get('historical_done_sha256')
        probe.require(proof.get('scope') == 'QUIESCENT_ONLY_REQUIRES_NEW_STOP'
                      and isinstance(pids, list) and 1 <= len(pids) <= 1024
                      and all(type(pid) is int and pid > 1 for pid in pids)
                      and len(pids) == len(set(pids)) and binding.get('runner_pid') in pids
                      and (digest is None or isinstance(digest, str) and re.fullmatch('[0-9a-f]{64}', digest))
                      and 'stop_confirmed' not in proof, 'RECOVERY_PI_PROOF_INVALID')
    return proof


def audit(state_dir, *, remote=remote_proof, lock_path=None):
    marker, digest, out = pending_record(state_dir)
    pids, binding, hashes = local_proof(marker, out)
    fd = probe.lock_existing(lock_path or Path.home() / '.cache/fire_robot_r5_laptop.lock')
    try:
        probe.absent(pids)
        probe.no_robot_processes()
        pi = remote(binding)
        # Inspect local groups again after the bounded network round trip.
        probe.absent(pids)
        probe.no_robot_processes()
        _, latest, _ = pending_record(state_dir)
        probe.require(latest == digest, 'RECOVERY_MARKER_CHANGED')
        return dict(marker_sha256=digest, session=marker['session'], packet=marker['packet'],
                    local=hashes, remote=pi, recheck_pids=pids, checked_mono=time.monotonic())
    finally:
        os.close(fd)


def archive_verified(state_dir, result, recovery_id, *, now=time.monotonic, lock_path=None):
    probe.require(re.fullmatch('[0-9a-f]{32}', recovery_id), 'RECOVERY_ID_INVALID')
    checked = result.get('checked_mono')
    probe.require(type(checked) in (int, float) and 0 <= now() - checked <= 5., 'RECOVERY_RESULT_STALE')
    marker, digest, _ = pending_record(state_dir)
    probe.require(digest == result.get('marker_sha256') and marker['session'] == result.get('session')
                  and marker['packet'] == result.get('packet'), 'RECOVERY_MARKER_CHANGED')
    history = Path(state_dir) / 'recoveries'
    history.mkdir(mode=0o700, exist_ok=True)
    probe.require(not history.is_symlink() and history.resolve().parent == Path(state_dir).resolve(),
                  'RECOVERY_ARCHIVE_PATH_INVALID')
    folder = history / recovery_id
    folder.mkdir(mode=0o700)
    atomic(folder / 'audit.json', dict(result, scope=result.get('scope', 'READ_ONLY_RETRY_AUDIT_NO_NEW_PHYSICAL_STOP_CLAIM')))
    # Audit must be saved before retiring the marker. Old receipts/logs remain
    # unchanged, including the failed TF/startup result and RC=2.
    _, latest, _ = pending_record(state_dir)
    probe.require(latest == digest, 'RECOVERY_MARKER_CHANGED')
    fd = None
    try:
        if "recheck_pids" in result:
            fd = probe.lock_existing(lock_path or Path.home() / '.cache/fire_robot_r5_laptop.lock')
            probe.absent(result['recheck_pids'])
            probe.no_robot_processes()
            probe.require(pending_record(state_dir)[1] == digest, 'RECOVERY_MARKER_CHANGED')
        os.replace(Path(state_dir) / 'pending.json', folder / 'pending.json')
    finally:
        if fd is not None:
            os.close(fd)


def failure_message(exc):
    """A useful persistent UI result without arbitrary file/SSH contents."""
    match = re.search(r'RECOVERY_[A-Z0-9_]+', str(exc)) if isinstance(exc, ValueError) else None
    code = match.group() if match else type(exc).__name__
    messages = {
        'RECOVERY_CONNECT_PI_WIFI_OR_LAN': 'Hãy kết nối WiFi hoặc LAN với Pi rồi kiểm tra lại',
        'RECOVERY_PI_SSH_FAILED': 'Chưa kết nối SSH được với Pi; kiểm tra kết nối rồi thử lại',
        'RECOVERY_STOP_NOT_CONFIRMED': 'Phiên cũ chưa có xác nhận dừng robot',
        'RECOVERY_LOCAL_CLEANUP_MISSING': 'Chưa có đủ bằng chứng tiến trình laptop đã thoát sạch',
        'RECOVERY_GROUP_NOT_CLEAN': 'Có tiến trình chưa thoát sạch; cần kiểm tra log phiên cũ',
        'RECOVERY_PROCESS_STILL_LIVE': 'Tiến trình của phiên cũ vẫn còn chạy',
        'RECOVERY_PROFILE_UNSUPPORTED': 'Loại phiên cũ chưa được hỗ trợ kiểm tra phục hồi',
        'RECOVERY_PI_BOOT_CHANGED': 'Pi đã khởi động lại; cần kiểm tra riêng trước khi mở phiên mới',
        'RECOVERY_ROBOT_PROCESS_OR_SNAPSHOT_UNKNOWN': 'Còn tiến trình robot hoặc chưa đọc được trạng thái tiến trình',
        'RECOVERY_PI_PROOF_INVALID': 'Bằng chứng Pi không hợp lệ; chưa thể xác minh dừng',
        'RECOVERY_NEW_STOP_NOT_CONFIRMED': 'Phiên xác minh mới chưa có đủ STOP và cleanup',
        'RECOVERY_PI_CLEANUP_MISSING': 'Cleanup Pi của phiên cũ có lỗi hoặc thiếu; dùng Xác minh dừng để lấy xác nhận mới',
        'RECOVERY_OWNERSHIP_MISSING': 'Thiếu hồ sơ sở hữu tiến trình; chưa thể xác minh an toàn phiên cũ',
        'RECOVERY_OWNERSHIP_INCOMPLETE': 'Hồ sơ khởi động dở chưa đủ để xác định tiến trình cũ',
        'RECOVERY_LOG_TOO_LARGE': 'Log phiên quá lớn để kiểm tra trong giới hạn; cần kiểm tra riêng',
        'FileNotFoundError': 'Thiếu file xác nhận của phiên cũ; cần kiểm tra log',
        'PermissionError': 'Không đọc hoặc lưu được kết quả; kiểm tra quyền truy cập',
        'BlockingIOError': 'Một phiên khác đang giữ quyền vận hành; hãy kết thúc phiên đó trước',
        'TimeoutExpired': 'Pi chưa trả lời trong thời gian kiểm tra; kiểm tra kết nối rồi thử lại',
    }
    # Prefer a known inner protocol cause; never display raw SSH/error text.
    if isinstance(exc, ValueError) and re.fullmatch(r'RECOVERY_[A-Z0-9_]+(?::RECOVERY_[A-Z0-9_]+)*', str(exc)):
        code = next((item for item in reversed(str(exc).split(':')) if item in messages), code)
    message = messages.get(code, 'Chưa đủ bằng chứng phiên cũ đã kết thúc sạch; xem log và mã lỗi')
    return 'Chưa thể mở phiên mới: ' + message + ' (' + code + ')'


class RecoveryAudit:
    def __init__(self, *, state_dir, emit, new_stop_out=None):
        self.id = uuid.uuid4().hex
        self.state_dir, self.emit = state_dir, emit
        self.new_stop_out = new_stop_out
        self.thread = threading.Thread(target=self.run, name='read-only-retry-audit', daemon=False)

    def start(self):
        self.thread.start()

    def run(self):
        try:
            if self.new_stop_out is None:
                result = audit(self.state_dir)
            else:
                from .stop_recovery_runner import audit_new_stop
                result = audit_new_stop(self.state_dir, self.new_stop_out)
            row = dict(result=result)
        except Exception as exc:
            row = dict(error=failure_message(exc))
        self.emit(dict(event='RECOVERY_FINISHED', recovery_id=self.id, **row))
