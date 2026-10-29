#!/usr/bin/env bash
# User-run only. Transfer/source build/static check; never launch ROS/devices/NTP.
set -Eeo pipefail
umask 077
host=${1:-10.42.0.185}
[[ "$host" == 10.42.0.185 || "$host" == 10.0.0.1 ]] || {
  echo 'Use Pi LAN 10.42.0.185 or Pi AP 10.0.0.1'
  exit 2
}

packet=0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0
archive_sha=f61a583ee605531484b1168b8a31ee0ba7c846522958ea26f3b907a1b373185f
remote_receipt="/home/pi/r5_staging/${packet:0:12}/pi_install_receipt.json"
evidence=$(cd -- "$(dirname -- "$0")" && pwd)
archive="$evidence/r5_model_29102026_source.tar.gz"
printf '%s  %s\n' "$archive_sha" "$archive" | sha256sum -c -

mkdir -p "$evidence/../model_29102026_sync_runs"
out=$(mktemp -d "$evidence/../model_29102026_sync_runs/sync_$(date +%Y%m%d_%H%M%S)_XXXXXX")
printf 'LOG_SYNC=%s\n' "$out"
remote_archive=/home/pi/r5_staging/r5_model_29102026_0c54d9b79765.tar.gz
ssh -o ConnectTimeout=8 "pi@$host" 'mkdir -p /home/pi/r5_staging'
scp -q -o ConnectTimeout=8 "$archive" "pi@$host:$remote_archive"
ssh -o ConnectTimeout=8 "pi@$host" bash -s <<'REMOTE' | tee "$out/pi_build_static.log"
set -Eeo pipefail
umask 077
python3 - <<'PY'
from pathlib import Path, PurePosixPath
import hashlib
import subprocess
import tarfile

packet = '0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0'
archive = Path('/home/pi/r5_staging/r5_model_29102026_0c54d9b79765.tar.gz')
stage = Path('/home/pi/r5_staging/0c54d9b79765/ros')
baseline = Path('/home/pi/NCHK_robotFirefighting_ROS_baseline')
sha = lambda data: hashlib.sha256(data).hexdigest()

assert sha(archive.read_bytes()) == 'f61a583ee605531484b1168b8a31ee0ba7c846522958ea26f3b907a1b373185f'
assert subprocess.check_output(
    ['git', '-C', str(baseline), 'rev-parse', 'HEAD'], text=True
).strip() == '6d8cf0f1b15776e695ae4e7868ca4eb1107837fd'
assert not subprocess.check_output(
    ['git', '-C', str(baseline), 'status', '--short'], text=True
)
with tarfile.open(archive, 'r:gz') as tar:
    rows = tar.getmembers()
    names = [member.name for member in rows]
    assert len(names) == len(set(names)) == 365
    assert all(member.isfile() for member in rows)
    for name in names:
        path = PurePosixPath(name)
        assert not path.is_absolute() and '..' not in path.parts and str(path) == name
    raw = tar.extractfile('operational_packet.sha256').read()
    assert sha(raw) == packet
    entries = {path: digest for digest, path in (
        line.split('  ', 1) for line in raw.decode().splitlines()
    )}
    assert len(entries) == 364
    assert set(names) == set(entries) | {'operational_packet.sha256'}
    for path, digest in entries.items():
        assert sha(tar.extractfile(path).read()) == digest, path
    if stage.parent.exists():
        assert stage.is_dir() and not stage.is_symlink()
        assert (stage / 'operational_packet.sha256').read_bytes() == raw
        for path, digest in entries.items():
            assert sha((stage / path).read_bytes()) == digest, path
    else:
        stage.mkdir(parents=True, mode=0o700)
        tar.extractall(stage)
    print('PI_SOURCE_PINS=364')
PY
source /opt/ros/humble/setup.bash
source /home/pi/NCHK_robotFirefighting_ROS_baseline/ros2_ws/install/setup.bash
cd /home/pi/r5_staging/0c54d9b79765/ros/ros2_ws
colcon build --packages-select fire_robot_bringup fire_robot_mission --symlink-install
source install/local_setup.bash
cd ..
python3 scripts/r5_packet.py
python3 scripts/r5_install_check.py --role pi > ../pi_install_receipt.json
python3 - <<'PY'
from pathlib import Path
import hashlib
import json

receipt = json.loads(Path('../pi_install_receipt.json').read_text())
assert receipt['status'] == 'STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED'
assert not receipt['errors']
assert receipt['checks']['source_packet_sha256'] == '0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0'
assert len(receipt['checks']['installed_sources']) == 103
for row in receipt['checks']['installed_sources']:
    assert hashlib.sha256(Path(row['path']).read_bytes()).hexdigest() == row['sha256']
print('PI_STATIC_HASH_PASS=1')
PY
REMOTE

scp -q -o ConnectTimeout=8 \
  "pi@$host:$remote_receipt" \
  "$out/pi_install_receipt.json"
python3 "$evidence/validate_pi_receipt.py" "$out/pi_install_receipt.json"
printf 'LOG_SYNC=%s\n' "$out"
