import hashlib
from pathlib import Path
import tempfile
import unittest
import sys
from types import SimpleNamespace
from unittest.mock import patch

from services.app_model import validate_model
from services.desktop_hooks import DesktopHooks


class AppModelTests(unittest.TestCase):
    def test_pin_and_missing_or_changed_weights(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'weights.pt'
            digest = hashlib.sha256(b'weights').hexdigest()
            with self.assertRaisesRegex(ValueError, 'MISSING'):
                validate_model(path, digest)
            path.write_bytes(b'weights')
            self.assertEqual(validate_model(path, digest), path)
            path.write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, 'HASH_MISMATCH'):
                validate_model(path, digest)
            link = path.parent / 'link.pt'
            link.symlink_to(path)
            with self.assertRaisesRegex(ValueError, 'MISSING'):
                validate_model(link, digest)

    def test_desktop_launch_overrides_only_app_yolo_without_mutating_original(self):
        hooks = DesktopHooks.__new__(DesktopHooks)
        hooks.operator = SimpleNamespace(command=['app-operator'])
        hooks.runtime = Path('/tmp/runtime')
        hooks.out = Path('/tmp/session')
        hooks.binding = SimpleNamespace(session='session')
        original = {'yolo': ['ros2', 'launch', 'perception'], 'operator': ['r5-operator'], 'rviz': ['rviz'],
                    'nav': ['ros2', 'launch', '--noninteractive', 'fire_robot_navigation', 'nav2_composed_shadow.launch.py']}
        original['mission'] = ['ros2', 'launch', '--noninteractive', 'fire_robot_mission', 'operational_stack_gate2.launch.py']
        hooks.app = Path('/tmp/app-only-model-test/desktop')
        with patch('services.app_model.validate_model', return_value=Path('/tmp/best_minireplay.pt')):
            result = hooks.commands(original)
        self.assertEqual(result['yolo'][-1], 'model_path:=/tmp/best_minireplay.pt')
        self.assertEqual(original['yolo'], ['ros2', 'launch', 'perception'])
        self.assertEqual(original['operator'], ['r5-operator'])

    def test_missing_new_model_cannot_block_mapping_or_stop_verification(self):
        from services import live_preflight
        modules = {
            'operational_entry': SimpleNamespace(packet_digest=lambda _: 'a'*64),
            'operational_preflight': SimpleNamespace(preflight=lambda *_: None, validate_gate2_profile=lambda *_: None),
            'r5_install_check': SimpleNamespace(inventory=lambda *_: {'status': 'STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED', 'errors': []}),
        }
        with patch.dict(sys.modules, modules), patch('services.app_model.validate_model', side_effect=ValueError('APP_YOLO_MODEL_MISSING')), \
             patch.object(live_preflight, 'check_existing_runner_lock'):
            for kind in ('mapping', 'verify_stop'):
                with patch.object(sys, 'argv', ['preflight', '/tmp', 'a'*64, kind]):
                    live_preflight.main()
            with patch.object(sys, 'argv', ['preflight', '/tmp', 'a'*64, 'fire']):
                with self.assertRaisesRegex(ValueError, 'APP_YOLO_MODEL_MISSING'):
                    live_preflight.main()
