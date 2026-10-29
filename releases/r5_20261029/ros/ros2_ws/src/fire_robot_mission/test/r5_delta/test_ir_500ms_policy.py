"""Delta: 5 Hz IR may age to 500 ms; other proof still has its old bound."""
from pathlib import Path

from fire_robot_mission.cycle_release import ReleaseInputs
from fire_robot_mission.ir_approach_live_core import IrApproachLiveCore
from fire_robot_mission.operational_session_core import OperationalSessionCore
from fire_robot_mission.operational_review import OperationalReview
from fire_robot_mission.pulse_session import PulseSession
from fire_robot_mission.pump_demo_core import PumpDemoCore


def test_ir_source_age_boundary_does_not_relax_vision():
    core = OperationalSessionCore()
    core.update('ir', {'ir_receive_age_ms': 450.0}, 1.0)
    assert core.get('ir', 1.05) is not None
    assert core.get('ir', 1.051) is None
    core.update('vision', {'source': {'age_ms': 301.0}}, 1.0)
    assert core.get('vision', 1.0) is None


def test_ir_receipt_boundary_across_live_pump_and_release():
    live = IrApproachLiveCore()
    live.inputs['ir'] = ({'sample': True}, 1.0)
    live.inputs['cmd'] = ({'sample': True}, 1.0)
    assert live._input('ir', 1.45) is not None
    assert live._input('ir', 1.501) is None
    assert live._input('cmd', 1.45) is None

    pulse = PulseSession()
    pulse.update('ir', {'sample': True}, 1.0)
    pulse.update('cmd', {'sample': True}, 1.0)
    assert pulse.get('ir', 1.45) is not None
    assert pulse.get('ir', 1.501) is None
    assert pulse.get('cmd', 1.45) is None

    pump = PumpDemoCore()
    pump.inputs['ir'] = ({'sample': True}, 1.0)
    pump.inputs['cmd'] = ({'sample': True}, 1.0)
    assert pump._input('ir', 1.45) is not None
    assert pump._input('ir', 1.501) is None
    assert pump._input('cmd', 1.45) is None

    release = ReleaseInputs()
    release.update('ir', {'sample': True}, 1.0)
    release.update('cmd', {'sample': True}, 1.0)
    assert release.get('ir', 1.45) is not None
    assert release.get('ir', 1.501) is None
    assert release.get('cmd', 1.45) is None


def test_installed_preview_uses_500ms_ir_only():
    yaml = (Path(__file__).resolve().parents[2] / 'config' / 'ir_range_preview.yaml').read_text()
    assert 'max_ir_age_ms: 500.0' in yaml
    assert 'max_image_age_ms: 300.0' in yaml


def test_post_spray_review_keeps_ir_500ms_and_drops_older_receipt():
    review = OperationalReview()
    review.update('ir', {'ir_receive_age_ms': 0.0}, 1.0)
    review.step(1.45, graph_ok=True)
    assert review.core.inputs['ir'][0]['ir_receive_age_ms'] > 400
    review.step(1.501, graph_ok=True)
    assert review.core.inputs['ir'][0] is None
