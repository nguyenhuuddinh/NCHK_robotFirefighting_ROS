from launch_ros.actions import Node
import os
import unittest
import xml.etree.ElementTree as ET

import importlib.util

pkg_share = os.path.join(os.path.dirname(__file__), '..')
launch_file = os.path.join(pkg_share, 'launch', 'nav2.launch.py')

spec = importlib.util.spec_from_file_location("nav2_launch", launch_file)
nav2_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav2_launch)


class TestNav2Invariants(unittest.TestCase):
    def test_command_route(self):
        ld = nav2_launch.generate_launch_description()
        nodes = [e for e in ld.entities if isinstance(e, Node)]

        for node in nodes:
            name = node.node_executable
            if isinstance(name, list) and len(name) > 0 and hasattr(name[0], 'text'):
                name = name[0].text

            remaps = getattr(node, '_Node__remappings', [])

            def get_text(subs):
                if not subs:
                    return ''
                return ''.join([s.text for s in subs if hasattr(s, 'text')])

            str_remaps = [(get_text(r[0]), get_text(r[1])) for r in remaps]

            if name == 'controller_server':
                has_cmd_vel_nav = any(r[0] == 'cmd_vel' and r[1] ==
                                      'cmd_vel_nav' for r in str_remaps)
                self.assertTrue(has_cmd_vel_nav,
                                "controller_server must remap cmd_vel to cmd_vel_nav")
            elif name == 'velocity_smoother':
                has_cmd_vel_nav = any(r[0] == 'cmd_vel' and r[1] ==
                                      'cmd_vel_nav' for r in str_remaps)
                has_cmd_vel_smoothed = any(
                    r[0] == 'cmd_vel_smoothed' and r[1] == '/cmd_vel_raw' for r in str_remaps)
                self.assertTrue(has_cmd_vel_nav,
                                "velocity_smoother must take cmd_vel_nav as cmd_vel")
                self.assertTrue(has_cmd_vel_smoothed,
                                "velocity_smoother must remap cmd_vel_smoothed to /cmd_vel_raw")
            elif name == 'behavior_server':
                has_cmd_vel_raw = any(r[0] == 'cmd_vel' and r[1] ==
                                      '/cmd_vel_raw' for r in str_remaps)
                self.assertTrue(has_cmd_vel_raw,
                                "behavior_server must remap cmd_vel to /cmd_vel_raw")

            if name in ['controller_server', 'velocity_smoother', 'behavior_server']:
                has_direct_cmd_vel = any(r[0] == 'cmd_vel' and r[1] ==
                                         'cmd_vel' for r in str_remaps)
                self.assertFalse(has_direct_cmd_vel,
                                 f"{name} must not publish directly to cmd_vel")

    def test_bt_no_reverse(self):
        bt_dir = os.path.join(pkg_share, 'behavior_trees')
        bts = ['navigate_to_pose_no_reverse.xml', 'navigate_through_poses_no_reverse.xml']
        for bt_file in bts:
            path = os.path.join(bt_dir, bt_file)
            tree = ET.parse(path)
            root = tree.getroot()
            nodes = [elem.tag for elem in root.iter()]
            self.assertNotIn('BackUp', nodes, f"BackUp found in {bt_file}")

    def test_tf_ownership(self):
        ld = nav2_launch.generate_launch_description()
        nodes = [e for e in ld.entities if isinstance(e, Node)]
        executables = []
        for n in nodes:
            name = n.node_executable
            if isinstance(name, list) and len(name) > 0 and hasattr(name[0], 'text'):
                name = name[0].text
            elif isinstance(name, str):
                pass
            else:
                continue
            executables.append(name)
        self.assertNotIn('robot_state_publisher', executables)
        self.assertNotIn('odom_to_tf_broadcaster', executables)
        self.assertNotIn('static_transform_publisher', executables)

    def _validate_artifacts(self, installed_paths):
        expected_paths = [
            'launch/nav2.launch.py',
            'launch/slam.launch.py',
            'config/nav2_params.yaml',
            'config/mapper_params_online_async.yaml',
            'config/slam.rviz',
            'behavior_trees/navigate_to_pose_no_reverse.xml',
            'behavior_trees/navigate_through_poses_no_reverse.xml',
            'package.xml'
        ]
        # Verify exact installed deliverables
        for p in installed_paths:
            self.assertIn(p, expected_paths, f"Unexpected installed file: {p}")
        for expected in expected_paths:
            self.assertIn(
                expected, installed_paths, f"Missing expected installed file: {expected}")

    def _get_managed_installed_paths(self, install_share):
        installed_paths = []
        gen_dirs = {'cmake', 'environment', 'hook'}
        gen_files = {
            'local_setup.bash', 'local_setup.dsv', 'local_setup.sh',
            'local_setup.zsh', 'package.bash', 'package.dsv', 'package.ps1',
            'package.sh', 'package.zsh'
        }
        for root, dirs, files in os.walk(install_share):
            dirs[:] = [d for d in dirs if d != '__pycache__']
            if os.path.relpath(root, install_share) == '.':
                dirs[:] = [d for d in dirs if d not in gen_dirs]

            for f in files:
                if f.endswith('.pyc') or f.endswith('.pyo'):
                    continue
                rel_path = os.path.relpath(os.path.join(root, f), install_share)
                if rel_path in gen_files:
                    continue
                installed_paths.append(rel_path)
        return installed_paths

    def test_installed_artifacts_allowlist(self):
        from ament_index_python.packages import get_package_share_directory
        install_share = get_package_share_directory('fire_robot_navigation')
        installed_paths = self._get_managed_installed_paths(install_share)
        self._validate_artifacts(installed_paths)

        # Check CMake static rules
        cmake_path = os.path.join(pkg_share, 'CMakeLists.txt')
        with open(cmake_path, 'r') as f:
            cmake_content = f.read()
        self.assertRegex(cmake_content, r'PATTERN\s+"\*\.pyc"\s+EXCLUDE')
        self.assertRegex(cmake_content, r'PATTERN\s+"\*\.pyo"\s+EXCLUDE')
        self.assertRegex(cmake_content, r'REGEX\s+"\(__pycache__\)"\s+EXCLUDE')

    def test_qa8_qa16_mutations(self):
        # QA8 & QA16: Prove unexpected artifacts are caught
        expected_paths = [
            'launch/nav2.launch.py',
            'launch/slam.launch.py',
            'config/nav2_params.yaml',
            'config/mapper_params_online_async.yaml',
            'config/slam.rviz',
            'behavior_trees/navigate_to_pose_no_reverse.xml',
            'behavior_trees/navigate_through_poses_no_reverse.xml',
            'package.xml'
        ]

        import tempfile
        import re

        mutations = [
            'launch/unexpected_debug.txt',
            'ament_unexpected_debug.txt',
            'local_setup.unexpected_debug.txt',
            'package.unexpected_debug.txt',
            'launch/cmake/unexpected.txt'
        ]

        for mutation in mutations:
            with self.subTest(mutation=mutation):
                with tempfile.TemporaryDirectory() as d:
                    # Setup base valid struct + pycache
                    os.makedirs(os.path.join(d, 'launch', '__pycache__'))
                    pyc_path = os.path.join(
                        d, 'launch', '__pycache__', 'nav2.launch.cpython-310.pyc'
                    )
                    with open(pyc_path, 'w') as f:
                        f.write('')
                    # Generate exact 3 dirs and 9 root files to prove valid fixtures
                    gen_dirs = ['cmake', 'environment', 'hook']
                    for gd in gen_dirs:
                        os.makedirs(os.path.join(d, gd))
                    gen_files = [
                        'local_setup.bash', 'local_setup.dsv', 'local_setup.sh',
                        'local_setup.zsh', 'package.bash', 'package.dsv', 'package.ps1',
                        'package.sh', 'package.zsh'
                    ]
                    for gf in gen_files:
                        with open(os.path.join(d, gf), 'w') as f:
                            f.write('')

                    for p in expected_paths:
                        os.makedirs(os.path.join(d, os.path.dirname(p)), exist_ok=True)
                        with open(os.path.join(d, p), 'w') as f:
                            f.write('')

                    # This should pass because the cache and gen files are ignored
                    managed_paths = self._get_managed_installed_paths(d)
                    self._validate_artifacts(managed_paths)

                    # Now add unexpected debug text
                    os.makedirs(os.path.join(d, os.path.dirname(mutation)), exist_ok=True)
                    with open(os.path.join(d, mutation), 'w') as f:
                        f.write('')

                    managed_paths_mutated = self._get_managed_installed_paths(d)
                    self.assertIn(mutation, managed_paths_mutated)
                    with self.assertRaisesRegex(AssertionError, re.escape(mutation)):
                        self._validate_artifacts(managed_paths_mutated)


if __name__ == '__main__':
    unittest.main()
