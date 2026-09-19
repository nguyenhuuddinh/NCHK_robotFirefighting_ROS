import os
import yaml
import copy
import unittest


class TestNav2Params(unittest.TestCase):
    def setUp(self):
        from ament_index_python.packages import get_package_share_directory
        self.pkg_share = get_package_share_directory('fire_robot_navigation')
        print(f"Resolved package_share: {self.pkg_share}")
        self.params_file = os.path.join(self.pkg_share, 'config', 'nav2_params.yaml')
        with open(self.params_file, 'r') as f:
            self.params = yaml.safe_load(f)

    def test_params_exist(self):
        self.assertIsNotNone(self.params)

    def test_duplicate_yaml(self):
        with open(self.params_file, 'r') as f:
            content = f.read()
        import re
        keys = re.findall(r'^(\w+):', content, re.MULTILINE)
        self.assertEqual(len(keys), len(set(keys)), "Duplicate top-level keys found in YAML")

    def _validate_params(self, params):
        import math

        # 1. Exact top-level blocks
        required_blocks = {
            'amcl', 'bt_navigator',
            'bt_navigator_navigate_through_poses_rclcpp_node',
            'bt_navigator_navigate_to_pose_rclcpp_node',
            'controller_server', 'local_costmap', 'global_costmap',
            'map_server', 'planner_server', 'smoother_server',
            'behavior_server', 'velocity_smoother'
        }
        self.assertEqual(
            set(params.keys()), required_blocks,
            "Exact YAML top-level blocks mismatch")

        # 2. Frame invariants
        amcl_params = params.get('amcl', {}).get('ros__parameters', {})
        self.assertEqual(amcl_params.get('global_frame_id'), 'map')
        self.assertEqual(amcl_params.get('odom_frame_id'), 'odom')
        self.assertEqual(amcl_params.get('base_frame_id'), 'base_link')
        self.assertEqual(amcl_params.get('scan_topic'), '/scan')
        for key in ['update_min_d', 'update_min_a']:
            self.assertIn(key, amcl_params, f"Missing AMCL motion threshold {key}")
            self.assertTrue(math.isfinite(amcl_params.get(key)))
            self.assertGreater(amcl_params.get(key), 0.0)
            self.assertLessEqual(amcl_params.get(key), 0.05)

        bt_params = params.get('bt_navigator', {}).get('ros__parameters', {})
        self.assertEqual(bt_params.get('global_frame'), 'map')
        self.assertEqual(bt_params.get('robot_base_frame'), 'base_link')
        self.assertEqual(bt_params.get('odom_topic'), '/odom')

        local_cm = params.get('local_costmap', {}).get('local_costmap', {})
        local_cm_p = local_cm.get('ros__parameters', {})
        self.assertEqual(local_cm_p.get('global_frame'), 'odom')
        self.assertEqual(local_cm_p.get('robot_base_frame'), 'base_link')
        self.assertEqual(local_cm_p.get('voxel_layer', {}).get('scan', {}).get('topic'), '/scan')

        global_cm = params.get('global_costmap', {}).get('global_costmap', {})
        global_cm_p = global_cm.get('ros__parameters', {})
        self.assertEqual(global_cm_p.get('global_frame'), 'map')
        self.assertEqual(global_cm_p.get('robot_base_frame'), 'base_link')
        self.assertEqual(
            global_cm_p.get('obstacle_layer', {}).get('scan', {}).get('topic'),
            '/scan')

        # 3. AMCL TF broadcast
        self.assertTrue(
            amcl_params.get('tf_broadcast', False),
            "AMCL must explicitly broadcast map->odom tf")

        # 4. Banned keys
        banned = [
            'slam_toolbox', 'watchdog', 'safety_watchdog', 'lidar',
            'camsense', 'raw_serial', 'serial_bridge', 'firmware'
        ]

        def walk_dict(d, path=""):
            for k, v in d.items():
                if isinstance(k, str):
                    for b in banned:
                        self.assertNotIn(b, k.lower(), f"Found banned key {k} at {path}")
                if isinstance(v, dict):
                    walk_dict(v, path + "." + str(k))
        walk_dict(params)

        controller = params.get('controller_server', {}).get('ros__parameters', {})
        rpp = controller.get('FollowPath', {})
        goal_checker = controller.get('general_goal_checker', {})
        progress_checker = controller.get('progress_checker', {})

        self.assertEqual(controller.get('controller_frequency'), 10.0)

        self.assertEqual(
            rpp.get('plugin'),
            'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
        self.assertNotIn('primary_controller', rpp)
        rpp_keys = [
            'desired_linear_vel', 'lookahead_dist', 'min_lookahead_dist',
            'max_lookahead_dist', 'lookahead_time',
            'min_approach_linear_velocity', 'approach_velocity_scaling_dist',
            'regulated_linear_scaling_min_radius',
            'regulated_linear_scaling_min_speed', 'cost_scaling_dist',
            'cost_scaling_gain', 'inflation_cost_scaling_factor',
            'max_allowed_time_to_collision_up_to_carrot',
            'rotate_to_heading_min_angle', 'rotate_to_heading_angular_vel',
            'max_angular_accel', 'transform_tolerance',
            'max_robot_pose_search_dist'
        ]
        for key in rpp_keys:
            self.assertIn(key, rpp, f"Missing RPP key {key}")
            self.assertTrue(math.isfinite(rpp[key]))
            self.assertGreater(rpp[key], 0.0)
        for key in [
            'use_velocity_scaled_lookahead_dist',
            'use_regulated_linear_velocity_scaling',
            'use_cost_regulated_linear_velocity_scaling',
            'use_collision_detection', 'use_rotate_to_heading',
            'use_interpolation'
        ]:
            self.assertIs(rpp.get(key), True)
        self.assertIs(rpp.get('allow_reversing'), False)
        self.assertLessEqual(rpp['min_lookahead_dist'], rpp['lookahead_dist'])
        self.assertLessEqual(rpp['lookahead_dist'], rpp['max_lookahead_dist'])
        self.assertLessEqual(rpp['rotate_to_heading_min_angle'], math.pi)
        self.assertLessEqual(rpp['min_approach_linear_velocity'], rpp['desired_linear_vel'])
        self.assertLessEqual(rpp['regulated_linear_scaling_min_speed'], rpp['desired_linear_vel'])

        for k in ['xy_goal_tolerance', 'yaw_goal_tolerance']:
            self.assertIn(k, goal_checker, f"Missing {k} in general_goal_checker")
            self.assertTrue(math.isfinite(goal_checker.get(k)))
            self.assertGreater(goal_checker.get(k), 0.0)

        for k in ['required_movement_radius', 'movement_time_allowance']:
            self.assertIn(k, progress_checker, f"Missing {k} in progress_checker")
            self.assertTrue(math.isfinite(progress_checker.get(k)))
            self.assertGreater(progress_checker.get(k), 0.0)
        self.assertLessEqual(
            progress_checker.get('required_movement_radius'),
            goal_checker.get('xy_goal_tolerance'))

        smoother = params.get('velocity_smoother', {}).get('ros__parameters', {})
        max_vel = smoother.get('max_velocity')
        min_vel = smoother.get('min_velocity')
        max_accel = smoother.get('max_accel')
        max_decel = smoother.get('max_decel')

        self.assertEqual(len(max_vel), 3)
        self.assertEqual(len(min_vel), 3)
        self.assertEqual(len(max_accel), 3)
        self.assertEqual(len(max_decel), 3)

        # math.isfinite on smoother
        for arr in [max_vel, min_vel, max_accel, max_decel]:
            for v in arr:
                self.assertTrue(math.isfinite(v))

        self.assertGreater(max_vel[0], 0.0)
        self.assertEqual(min_vel[0], 0.0)
        self.assertEqual(max_vel[1], 0.0)
        self.assertEqual(min_vel[1], 0.0)
        self.assertEqual(max_accel[1], 0.0)
        self.assertEqual(max_decel[1], 0.0)
        self.assertLess(max_decel[0], 0.0)
        self.assertLess(max_decel[2], 0.0)

        self.assertEqual(max_vel[0], rpp['desired_linear_vel'])
        self.assertEqual(min_vel[2], -max_vel[2])
        self.assertEqual(max_vel[2], rpp['rotate_to_heading_angular_vel'])
        self.assertEqual(max_accel[2], rpp['max_angular_accel'])

        behaviors = params.get('behavior_server', {}).get('ros__parameters', {})
        for k in ['max_rotational_vel', 'min_rotational_vel', 'rotational_acc_lim']:
            self.assertIn(k, behaviors, f"Missing required key {k} in behavior_server")
            self.assertTrue(math.isfinite(behaviors.get(k)))
        self.assertGreaterEqual(behaviors.get('min_rotational_vel'), 0.0)
        self.assertLessEqual(
            behaviors.get('min_rotational_vel'),
            behaviors.get('max_rotational_vel')
        )
        self.assertGreater(behaviors.get('rotational_acc_lim'), 0.0)

        # Consistency between RPP, smoother and recovery Spin.
        self.assertEqual(rpp['desired_linear_vel'], max_vel[0])
        self.assertEqual(rpp['rotate_to_heading_angular_vel'], max_vel[2])
        self.assertEqual(max_vel[2], behaviors['max_rotational_vel'])
        self.assertEqual(rpp['max_angular_accel'], max_accel[2])
        self.assertEqual(max_accel[2], behaviors['rotational_acc_lim'])

        planner = params.get('planner_server', {}).get('ros__parameters', {})
        self.assertEqual(planner.get('expected_planner_frequency'), 2.0)
        self.assertEqual(global_cm_p.get('update_frequency'), 2.0)
        self.assertEqual(global_cm_p.get('publish_frequency'), 2.0)

        # Check plugin validity (must be strings)
        self.assertTrue(isinstance(rpp.get('plugin'), str))

    def test_baseline(self):
        self._validate_params(self.params)

    def test_operator_approved_motion_envelope(self):
        controller = self.params['controller_server']['ros__parameters']
        rpp = controller['FollowPath']
        goal_checker = controller['general_goal_checker']
        progress_checker = controller['progress_checker']
        behaviors = self.params['behavior_server']['ros__parameters']
        smoother = self.params['velocity_smoother']['ros__parameters']

        self.assertEqual(controller['controller_frequency'], 10.0)
        self.assertEqual(rpp['desired_linear_vel'], 0.10)
        self.assertEqual(rpp['rotate_to_heading_angular_vel'], 1.0)
        self.assertEqual(rpp['rotate_to_heading_min_angle'], 0.174533)
        self.assertEqual(rpp['lookahead_dist'], 0.20)
        self.assertEqual(rpp['min_lookahead_dist'], 0.15)
        self.assertEqual(rpp['max_lookahead_dist'], 0.25)
        self.assertIs(rpp['use_collision_detection'], True)
        self.assertIs(rpp['allow_reversing'], False)
        self.assertEqual(goal_checker['xy_goal_tolerance'], 0.08)
        self.assertEqual(goal_checker['yaw_goal_tolerance'], 0.15)
        self.assertEqual(progress_checker['required_movement_radius'], 0.05)
        self.assertEqual(progress_checker['movement_time_allowance'], 25.0)
        self.assertEqual(behaviors['min_rotational_vel'], 0.8)
        self.assertEqual(behaviors['max_rotational_vel'], 1.0)
        self.assertEqual(smoother['max_velocity'], [0.10, 0.0, 1.0])
        self.assertEqual(smoother['min_velocity'], [0.0, 0.0, -1.0])

    def test_qa10_mutations(self):
        import copy

        # RPP: desired speed outside the velocity smoother envelope.
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] = 1.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # RPP: collision detection must remain enabled.
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath']['use_collision_detection'] = False
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # QA10-2: oversized negative angular minimum
        p = copy.deepcopy(self.params)
        p['velocity_smoother']['ros__parameters']['min_velocity'][2] = -999.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 1. Negative RPP lookahead
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath']['lookahead_dist'] = -0.1
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 2. NaN RPP lookahead
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath']['lookahead_dist'] = float('nan')
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 3. generic slam block (violates exact top-level blocks)
        p = copy.deepcopy(self.params)
        p['slam'] = {}
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 4. AMCL TF broadcast disabled
        p = copy.deepcopy(self.params)
        p['amcl']['ros__parameters']['tf_broadcast'] = False
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 5. Banned key
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['slam_toolbox_plugin'] = 'plugin'
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 6. Negative general_goal_checker tolerances (QA8)
        p = copy.deepcopy(self.params)
        goal_checker = p['controller_server']['ros__parameters']['general_goal_checker']
        goal_checker['xy_goal_tolerance'] = -0.1
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 7. minimum approach velocity > desired velocity
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath'][
            'min_approach_linear_velocity'] = 999.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 8. behavior min > max (QA8)
        p = copy.deepcopy(self.params)
        p['behavior_server']['ros__parameters']['min_rotational_vel'] = 999.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 9. negative RPP angular acceleration
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath']['max_angular_accel'] = -1.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 10. angular acceleration mismatch
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath']['max_angular_accel'] = 1.0
        p['velocity_smoother']['ros__parameters']['max_accel'][2] = 2.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 11. invalid smoother angular bounds (QA8)
        p = copy.deepcopy(self.params)
        p['velocity_smoother']['ros__parameters']['min_velocity'][2] = 1.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

    def _validate_bt_contract(self, params_dict, bt_files_content):
        # 1. Extract all tags
        import xml.etree.ElementTree as ET
        all_tags = set()
        for content in bt_files_content:
            root = ET.fromstring(content)
            for elem in root.iter():
                all_tags.add(elem.tag)
                if elem.tag == 'RateController':
                    self.assertEqual(float(elem.attrib['hz']), 2.0)
                if elem.tag == 'Wait':
                    self.assertEqual(float(elem.attrib['wait_duration']), 2.0)

        # 2. Builtin and custom tags check
        builtins = {'root', 'BehaviorTree', 'ReactiveSequence', 'ReactiveFallback', 'Sequence'}
        mapping = {
            'ComputePathToPose': 'nav2_compute_path_to_pose_action_bt_node',
            'ComputePathThroughPoses': 'nav2_compute_path_through_poses_action_bt_node',
            'FollowPath': 'nav2_follow_path_action_bt_node',
            'Spin': 'nav2_spin_action_bt_node',
            'Wait': 'nav2_wait_action_bt_node',
            'ClearEntireCostmap': 'nav2_clear_costmap_service_bt_node',
            'GoalUpdated': 'nav2_goal_updated_condition_bt_node',
            'RateController': 'nav2_rate_controller_bt_node',
            'RecoveryNode': 'nav2_recovery_node_bt_node',
            'PipelineSequence': 'nav2_pipeline_sequence_bt_node',
            'RoundRobin': 'nav2_round_robin_node_bt_node',
            'RemovePassedGoals': 'nav2_remove_passed_goals_action_bt_node',
        }

        bt_nav = params_dict.get('bt_navigator', {}).get('ros__parameters', {})
        bt_plugins = bt_nav.get('plugin_lib_names', [])

        # Check no reverse tags in XML
        for tag in all_tags:
            reverse_tags = ['BackUp', 'DriveOnHeading', 'AssistedTeleop']
            self.assertNotIn(tag, reverse_tags, f"Reverse tag found: {tag}")
            if tag in builtins:
                continue
            self.assertIn(tag, mapping, f"Unknown XML tag: {tag}")
            lib_name = mapping[tag]
            self.assertIn(lib_name, bt_plugins, f"Missing plugin for tag {tag}: {lib_name}")

            # verify library exists
            from ament_index_python.packages import get_package_prefix
            nav2_bt_prefix = get_package_prefix('nav2_behavior_tree')
            lib_path = os.path.join(nav2_bt_prefix, 'lib', f'lib{lib_name}.so')
            self.assertTrue(os.path.exists(lib_path), f"Library not found: {lib_path}")

        # Check no reverse plugins in params
        behaviors = params_dict.get('behavior_server', {}).get('ros__parameters', {})
        plugins = behaviors.get('behavior_plugins', [])
        self.assertNotIn('backup', plugins)
        self.assertNotIn('drive_on_heading', plugins)
        self.assertNotIn('assisted_teleop', plugins)

        banned_bt_plugins = [
            'nav2_back_up_action_bt_node',
            'nav2_drive_on_heading_bt_node',
            'nav2_assisted_teleop_action_bt_node',
            'nav2_back_up_cancel_bt_node',
            'nav2_drive_on_heading_cancel_bt_node',
            'nav2_assisted_teleop_cancel_bt_node'
        ]
        for p in banned_bt_plugins:
            self.assertNotIn(p, bt_plugins, f"Reverse plugin found: {p}")

    def _get_bt_files_content(self, pkg_share=None):
        if pkg_share is None:
            pkg_share = self.pkg_share
        bt_files = [
            'navigate_to_pose_no_reverse.xml',
            'navigate_through_poses_no_reverse.xml'
        ]
        contents = []
        for bt_file in bt_files:
            xml_path = os.path.join(pkg_share, 'behavior_trees', bt_file)
            with open(xml_path, 'r') as f:
                contents.append(f.read())
        return contents

    def test_no_reverse(self):
        # Baseline validation
        bt_contents = self._get_bt_files_content()
        self._validate_bt_contract(self.params, bt_contents)

    def test_qa20_bt_mutations(self):
        import tempfile
        import shutil

        # Mutation 1 & 2: TemporaryDirectory copy installed XML/YAML and mutate
        with tempfile.TemporaryDirectory() as d:
            os.makedirs(os.path.join(d, 'config'))
            os.makedirs(os.path.join(d, 'behavior_trees'))

            yaml_dest = os.path.join(d, 'config', 'nav2_params.yaml')
            shutil.copy(self.params_file, yaml_dest)

            bt_files = [
                'navigate_to_pose_no_reverse.xml',
                'navigate_through_poses_no_reverse.xml'
            ]
            for bt_file in bt_files:
                src = os.path.join(self.pkg_share, 'behavior_trees', bt_file)
                dst = os.path.join(d, 'behavior_trees', bt_file)
                shutil.copy(src, dst)

            # Mutate XML copy (unknown tag)
            xml_mutate_path = os.path.join(d, 'behavior_trees', 'navigate_to_pose_no_reverse.xml')
            with open(xml_mutate_path, 'r') as f:
                content = f.read()
            with open(xml_mutate_path, 'w') as f:
                f.write(content.replace('<GoalUpdated/>', '<GoalUpdated/><UnknownInstalledTag/>'))

            # Load and validate XML mutation
            with open(yaml_dest, 'r') as f:
                loaded_params = yaml.safe_load(f)
            mutated_bt_contents = self._get_bt_files_content(d)
            with self.assertRaisesRegex(AssertionError, "Unknown XML tag: UnknownInstalledTag"):
                self._validate_bt_contract(loaded_params, mutated_bt_contents)

            # Revert XML for next mutation
            with open(xml_mutate_path, 'w') as f:
                f.write(content)

            # Mutate YAML copy (remove plugin)
            with open(yaml_dest, 'r') as f:
                yaml_content = f.read()
            yaml_content = yaml_content.replace('- nav2_goal_updated_condition_bt_node', '')
            with open(yaml_dest, 'w') as f:
                f.write(yaml_content)

            # Load and validate YAML mutation
            with open(yaml_dest, 'r') as f:
                loaded_params = yaml.safe_load(f)
            clean_bt_contents = self._get_bt_files_content(d)
            with self.assertRaisesRegex(AssertionError, "Missing plugin for tag GoalUpdated"):
                self._validate_bt_contract(loaded_params, clean_bt_contents)

        bt_contents = self._get_bt_files_content()

        # Mutation 3: reverse tag in XML
        bad_bt_contents2 = [bt_contents[0].replace('<GoalUpdated/>', '<GoalUpdated/><BackUp/>')]
        with self.assertRaisesRegex(AssertionError, "Reverse tag found: BackUp"):
            self._validate_bt_contract(self.params, bad_bt_contents2)

        # Mutation 4-6: deep copy independent mutations for 3 cancel plugins
        cancel_plugins = [
            'nav2_back_up_cancel_bt_node',
            'nav2_drive_on_heading_cancel_bt_node',
            'nav2_assisted_teleop_cancel_bt_node'
        ]
        for cp in cancel_plugins:
            p_mut = copy.deepcopy(self.params)
            p_mut_plugins = p_mut['bt_navigator']['ros__parameters']['plugin_lib_names']
            p_mut_plugins.append(cp)
            with self.assertRaisesRegex(AssertionError, cp):
                self._validate_bt_contract(p_mut, bt_contents)

    def test_plugins_validity(self):
        # controller
        controller = self.params.get('controller_server', {}).get('ros__parameters', {})
        self.assertEqual(
            controller.get('progress_checker', {}).get('plugin'),
            'nav2_controller::SimpleProgressChecker'
        )
        self.assertEqual(
            controller.get('general_goal_checker', {}).get('plugin'),
            'nav2_controller::SimpleGoalChecker'
        )
        self.assertEqual(
            controller.get('FollowPath', {}).get('plugin'),
            'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
        )

        from ament_index_python.packages import get_package_prefix
        rpp_prefix = get_package_prefix('nav2_regulated_pure_pursuit_controller')
        rpp_library = os.path.join(
            rpp_prefix, 'lib', 'libnav2_regulated_pure_pursuit_controller.so')
        self.assertTrue(os.path.exists(rpp_library), rpp_library)

        # planner
        planner = self.params.get('planner_server', {}).get('ros__parameters', {})
        self.assertEqual(
            planner.get('GridBased', {}).get('plugin'),
            'nav2_navfn_planner/NavfnPlanner'
        )

        # smoother
        smoother = self.params.get('smoother_server', {}).get('ros__parameters', {})
        self.assertEqual(
            smoother.get('simple_smoother', {}).get('plugin'),
            'nav2_smoother::SimpleSmoother'
        )

        # behaviors
        behaviors = self.params.get('behavior_server', {}).get('ros__parameters', {})
        self.assertEqual(
            behaviors.get('spin', {}).get('plugin'),
            'nav2_behaviors/Spin'
        )
        self.assertEqual(
            behaviors.get('wait', {}).get('plugin'),
            'nav2_behaviors/Wait'
        )

    def test_no_hardcoded_map(self):
        map_server = self.params.get('map_server', {}).get('ros__parameters', {})
        self.assertEqual(map_server.get('yaml_filename'), '')

    def test_velocity_smoother_deadband(self):
        smoother = self.params.get('velocity_smoother', {}).get('ros__parameters', {})
        deadband = smoother.get('deadband_velocity')
        self.assertEqual(deadband, [0.0, 0.0, 0.0])


if __name__ == '__main__':
    unittest.main()
