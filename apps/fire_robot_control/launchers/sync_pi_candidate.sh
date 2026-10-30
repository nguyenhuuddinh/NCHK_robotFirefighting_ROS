#!/usr/bin/env bash
# Explicit, user-run static sync only: source transfer/build/hash, no ROS
# launch, NTP operation, firmware flash, or actuator command.
set -Eeo pipefail
export PYTHONDONTWRITEBYTECODE=1
umask 077
app_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
artifact="${1:-}"
host="${2:-10.42.0.185}"
[[ -n "$artifact" && ( "$host" == 10.42.0.185 || "$host" == 10.0.0.1 ) ]] || {
  printf '%s\n' 'Cách dùng: sync_pi_candidate.sh <artifact> [10.42.0.185|10.0.0.1]' >&2
  exit 2
}
artifact="$(cd -- "$artifact" && pwd)"
export PYTHONPATH="$artifact/ros/desktop:${PYTHONPATH:-}"
packet="$(/usr/bin/python3 -c 'import sys; from services.package_candidate import verify_artifact; print(verify_artifact(sys.argv[1])["packet"])' "$artifact")"
archive_sha="$(/usr/bin/python3 -c 'import sys; from services.package_candidate import verify_artifact; print(verify_artifact(sys.argv[1])["archive_sha256"])' "$artifact")"
[[ "$packet" =~ ^[0-9a-f]{64}$ && "$archive_sha" =~ ^[0-9a-f]{64}$ ]] || exit 2
/usr/bin/python3 -B -m services.packet_archive "$artifact/ros" --verify-archive "$artifact/source.tar.gz" --expected-packet "$packet"
state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/fire_robot_control"
mkdir -p "$state_dir/pi_receipts" "$state_dir/sync_logs"
out="$(mktemp -d "$state_dir/sync_logs/sync_XXXXXX")"
printf 'LOG_SYNC=%s\n' "$out"
remote_archive="/home/pi/fire_robot_desktop/incoming/$packet.tar.gz"
ssh -o ConnectTimeout=8 "pi@$host" 'mkdir -p -m 700 /home/pi/fire_robot_desktop/incoming'
scp -q -o ConnectTimeout=8 "$artifact/source.tar.gz" "pi@$host:$remote_archive"
ssh -o ConnectTimeout=8 "pi@$host" bash -s -- "$packet" "$archive_sha" <<'REMOTE' | tee "$out/pi_build_static.log"
set -Eeo pipefail
umask 077
packet=$1
archive_sha=$2
[[ "$packet" =~ ^[0-9a-f]{64}$ && "$archive_sha" =~ ^[0-9a-f]{64}$ ]]
export FIRE_ROBOT_SYNC_PACKET="$packet" FIRE_ROBOT_SYNC_ARCHIVE_SHA="$archive_sha"
python3 - <<'PY'
import hashlib, os, re, subprocess, tarfile
from pathlib import Path, PurePosixPath

packet = os.environ['FIRE_ROBOT_SYNC_PACKET']
expected_archive = os.environ['FIRE_ROBOT_SYNC_ARCHIVE_SHA']
base = Path('/home/pi/fire_robot_desktop')
archive = base / 'incoming' / (packet + '.tar.gz')
stage = base / packet[:12] / 'ros'
baseline = Path('/home/pi/NCHK_robotFirefighting_ROS_baseline')
sha = lambda data: hashlib.sha256(data).hexdigest()
if sha(archive.read_bytes()) != expected_archive:
    raise ValueError('ARCHIVE_HASH_MISMATCH')
if subprocess.check_output(['git', '-C', str(baseline), 'rev-parse', 'HEAD'], text=True).strip() != '6d8cf0f1b15776e695ae4e7868ca4eb1107837fd':
    raise ValueError('PI_BASELINE_COMMIT_MISMATCH')
if subprocess.check_output(['git', '-C', str(baseline), 'status', '--short'], text=True):
    raise ValueError('PI_BASELINE_DIRTY')
pattern = '[s]erial_bridge_node|[n]ative_mjpeg_camera|[s]afety_watchdog|[r]5_startup_health|[f]ire_robot_operator|[r]5_pi_session.py|[a]pp_pi_entry.py'
snapshot = subprocess.run(['pgrep', '-af', pattern], capture_output=True, text=True, check=False)
if snapshot.returncode != 1:
    raise ValueError('PI_ACTIVE_OR_PROCESS_CHECK_FAILED')
with tarfile.open(archive, 'r:gz') as tar:
    rows = tar.getmembers()
    names = [member.name for member in rows]
    if len(names) != len(set(names)) or any(not member.isfile() for member in rows):
        raise ValueError('ARCHIVE_DUPLICATE_OR_NONREGULAR')
    for name in names:
        p = PurePosixPath(name)
        if p.is_absolute() or '..' in p.parts or str(p) != name or '\\' in name:
            raise ValueError('ARCHIVE_UNSAFE_PATH')
    raw = tar.extractfile('operational_packet.sha256').read()
    if sha(raw) != packet:
        raise ValueError('MANIFEST_HASH_MISMATCH')
    entries = {}
    for line in raw.decode().splitlines():
        digest, name = line.split('  ', 1)
        if not re.fullmatch('[0-9a-f]{64}', digest) or name in entries:
            raise ValueError('MANIFEST_ENTRY_INVALID')
        entries[name] = digest
    if set(names) != set(entries) | {'operational_packet.sha256'}:
        raise ValueError('ARCHIVE_MEMBER_MISMATCH')
    for name, digest in entries.items():
        if sha(tar.extractfile(name).read()) != digest:
            raise ValueError('ARCHIVE_FILE_HASH_MISMATCH:' + name)
    if stage.parent.exists():
        if stage.parent.is_symlink() or not stage.is_dir() or stage.is_symlink():
            raise ValueError('PI_EXISTING_STAGE_INVALID')
        if (stage / 'operational_packet.sha256').read_bytes() != raw:
            raise ValueError('PI_EXISTING_STAGE_PACKET_MISMATCH')
        for name, digest in entries.items():
            p = stage / name
            if p.is_symlink() or not p.resolve().is_relative_to(stage.resolve()) or sha(p.read_bytes()) != digest:
                raise ValueError('PI_EXISTING_STAGE_FILE_MISMATCH:' + name)
    else:
        stage.mkdir(mode=0o700, parents=True)
        tar.extractall(stage)
print('PI_SOURCE_PINS=' + str(len(entries)))
PY
source /opt/ros/humble/setup.bash
source /home/pi/NCHK_robotFirefighting_ROS_baseline/ros2_ws/install/setup.bash
stage="/home/pi/fire_robot_desktop/${packet:0:12}/ros"
cd "$stage/ros2_ws"
colcon build --packages-select fire_robot_bringup fire_robot_mission --symlink-install
source install/local_setup.bash
cd "$stage"
PYTHONPATH="$stage/desktop:$stage/scripts" python3 -B -m services.packet_archive "$stage"
python3 scripts/r5_install_check.py --role pi > "../pi_install_receipt.json"
python3 - <<'PY'
import hashlib, json, os
from pathlib import Path
receipt = json.loads(Path('../pi_install_receipt.json').read_text())
if (receipt['status'] != 'STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED' or receipt['errors']
        or receipt['checks']['source_packet_sha256'] != os.environ['FIRE_ROBOT_SYNC_PACKET']
        or len(receipt['checks']['installed_sources']) != 103):
    raise ValueError('PI_STATIC_RECEIPT_MISMATCH')
for row in receipt['checks']['installed_sources']:
    if hashlib.sha256(Path(row['path']).read_bytes()).hexdigest() != row['sha256']:
        raise ValueError('PI_INSTALLED_HASH_MISMATCH')
print('PI_STATIC_HASH_PASS=1')
PY
REMOTE
scp -q -o ConnectTimeout=8 "pi@$host:/home/pi/fire_robot_desktop/${packet:0:12}/pi_install_receipt.json" "$out/pi_install_receipt.json"
/usr/bin/python3 - "$out/pi_install_receipt.json" "$state_dir/pi_receipts/$packet.json" "$packet" <<'PY'
import os, sys
from pathlib import Path
from services.pi_receipt import validate_pi_receipt
source, target, packet = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
validate_pi_receipt(source, packet)
data = source.read_bytes()
try:
    fd = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
except FileExistsError:
    validate_pi_receipt(target, packet)
    if target.read_bytes() != data:
        raise ValueError('EXISTING_PI_RECEIPT_DIFFERS; preserve and inspect both receipts')
else:
    with os.fdopen(fd, 'wb') as stream:
        stream.write(data)
print('PI_STATIC_RECEIPT_VALID=1')
PY
printf 'LOG_SYNC=%s\n' "$out"
