"""No ROS init: real health pose validation plus a fake AMCL Empty service."""
from types import SimpleNamespace as N
import copy
import pytest
from services.app_pose_refresh import PoseRefresh
from services.fire_health_entry import instrument


class Future:
    complete = False
    canceled = False
    def done(self): return self.complete
    def result(self): return N()
    def cancel(self): self.canceled = True


class Client:
    ready = True
    def __init__(self): self.sent = []; self.removed = []
    def service_is_ready(self): return self.ready
    def call_async(self, request):
        f = Future(); self.sent.append((request, f)); return f
    def remove_pending_request(self, future): self.removed.append(future)


def proof():
    return (dict(sample_mono=10., checks={'io': True}, stable_s={'io': 2.}),
        dict.fromkeys(('pose_source', 'initial_pose_seen', 'pose_after_initial',
                      'map_tf', 'action_ready', 'lifecycle', 'identity_present'), True)
        | {'pose_valid': False})


@pytest.mark.parametrize('missing', ['io', 'io_unstable', 'sample_stale', 'sample_future',
    'pose_source', 'initial_pose_seen', 'pose_after_initial', 'map_tf', 'action_ready',
    'lifecycle', 'identity_present', 'pose_valid', 'pose_recent', 'pose_absent'])
def test_missing_proof_never_requests_localization_update(missing):
    client = Client(); policy = PoseRefresh(client, lambda: N())
    row, detail = proof(); pose_at = 8.
    if missing == 'io': row['checks']['io'] = False
    elif missing == 'io_unstable': row['stable_s']['io'] = .9
    elif missing == 'sample_stale': row['sample_mono'] = 9.
    elif missing == 'sample_future': row['sample_mono'] = 11.
    elif missing == 'pose_valid': detail['pose_valid'] = True
    elif missing == 'pose_recent': pose_at = 9.5
    elif missing == 'pose_absent': pose_at = None
    else: detail[missing] = False
    policy.step(row, detail, pose_at, 10.)
    assert not client.sent and policy.report()['motion_authorized'] is False


def test_one_pending_request_timeout_backoff_and_retry_never_mutate_health():
    client = Client(); policy = PoseRefresh(client, lambda: N())
    row, detail = proof(); before = copy.deepcopy((row, detail))
    policy.step(row, detail, 8., 10.)
    assert len(client.sent) == 1
    f = client.sent[0][1]
    for now in (10.1, 11., 12.1, 14.9):
        row['sample_mono'] = now
        policy.step(row, detail, 8., now)
    assert f.canceled and client.removed == [f] and len(client.sent) == 1
    row['sample_mono'] = 15.; policy.step(row, detail, 8., 15.)
    assert len(client.sent) == 2
    row['sample_mono'] = before[0]['sample_mono']
    assert (row, detail) == before
    assert policy.report()['response_is_pose_proof'] is False


def test_unavailable_or_failed_service_keeps_pose_invalid():
    client = Client(); client.ready = False
    policy = PoseRefresh(client, lambda: N()); row, detail = proof()
    policy.step(row, detail, 8., 10.)
    assert not client.sent and policy.state == 'SERVICE_UNAVAILABLE'
    client.ready = True
    def fail(_): raise RuntimeError('fixture')
    client.call_async = fail
    row['sample_mono'] = 15.
    policy.step(row, detail, 8., 15.)
    assert policy.state == 'REQUEST_FAILED' and detail['pose_valid'] is False


def test_real_R5_pose_callback_not_service_ack_is_the_only_readiness_evidence(monkeypatch):
    from fire_robot_mission.startup_health import HealthCore, StartupHealthNode
    from geometry_msgs.msg import PoseWithCovarianceStamped
    now = [2.]; client = Client(); rows = []
    # Keep the real HealthCore and real R5 pose() implementation.
    class Base:
        def __init__(self):
            self.core = HealthCore('fixture', 'laptop'); self.core.initial_pose_at = 1.
            self.stamp_age = .7
        def create_client(self, typ, path):
            assert path == '/request_nomotion_update'
            return client
        def age(self, msg): return self.stamp_age
        pose = StartupHealthNode.pose
        def sample(self):
            from fire_robot_mission.operational_contract import FIRMWARE_ELF_SHA256
            self.core.update('identity', dict(schema_version=1, bridge_instance_id='ab'*16,
                connection_epoch=1, elf_sha256=FIRMWARE_ELF_SHA256, frame_protocol=2,
                source='SERIAL_V2_BOOT_WITH_LIVE_STATE', valid=True, session_ready=True,
                telemetry_healthy=True, state_age_ms=10, state_esp_ms=100), now[0])
            self.core.update('actuator', dict(schema_version=1, mode='COMMAND_TELEMETRY_ONLY',
                pump_pwm_ready=True, home_command_completed_count=0, receive_age_ms=10,
                pump_pwm_commanded_on=False, servo_pwm_commanded_on=False, servo_phase='IDLE'), now[0])
            for name, value in (('odom', [0.]*6), ('cmd', [0.]*6), ('scan', True)):
                self.core.update(name, value, now[0])
            return self.core.report(now[0], sources=dict(io=True, pose=True), tf_ok=True,
                odom_tf_ok=True, action_ready=True, safety_ok=True, lifecycle_ok=True)
    monkeypatch.setattr('time.monotonic', lambda: now[0])
    node = instrument(Base, N(due=lambda: True, emit=rows.append), 'fixture', 'packet')()
    message = PoseWithCovarianceStamped(); message.header.frame_id = 'map'
    message.pose.pose.orientation.w = 1.
    node.pose(message)
    assert not node.sample()['checks']['pose']
    for at in (2.2, 2.4, 2.6, 2.8, 3., 3.1):
        now[0] = at; assert not node.sample()['checks']['pose']
    assert len(client.sent) == 1 and rows[-1]['pose_sample']['stamp_ok'] is False
    client.sent[0][1].complete = True
    now[0] = 3.2; assert not node.sample()['checks']['pose']
    assert node._app_pose_refresh.state == 'WAIT_ACTUAL_POSE'
    now[0] = 3.3; node.stamp_age = .1; node.pose(message)
    assert node.sample()['checks']['pose']
    assert rows[-1]['pose_sample']['policy_valid'] is True
    assert node._app_pose_refresh.state == 'VALID_POSE_OBSERVED'
    # R5 short-circuits before reading age on a wrong frame. Never report the
    # preceding good message's age as evidence about this malformed sample.
    message.header.frame_id = 'wrong_frame'
    node.pose(message)
    assert node._app_pose_detail['frame_ok'] is False
    assert node._app_pose_detail['age_s'] is None
    assert node._app_pose_detail['policy_valid'] is False
