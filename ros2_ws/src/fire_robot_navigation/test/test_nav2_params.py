import os
import yaml
import unittest


class TestNav2Params(unittest.TestCase):
    def setUp(self):
        self.params_file = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml')
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
        dwb = controller.get('FollowPath', {})
        goal_checker = controller.get('general_goal_checker', {})

        # DWB FollowPath.xy_goal_tolerance (QA9)
        self.assertIn('xy_goal_tolerance', dwb, "Missing xy_goal_tolerance in FollowPath")
        self.assertTrue(math.isfinite(dwb.get('xy_goal_tolerance')))
        self.assertGreater(dwb.get('xy_goal_tolerance'), 0.0)

        # math.isfinite on dwb
        dwb_keys = [
            'max_vel_x', 'min_vel_x', 'max_vel_y', 'min_vel_y', 'max_vel_theta', 'min_speed_theta',
            'min_speed_xy', 'max_speed_xy', 'acc_lim_x', 'acc_lim_y', 'acc_lim_theta',
            'decel_lim_x', 'decel_lim_y', 'decel_lim_theta', 'trans_stopped_velocity'
        ]
        for k in dwb_keys:
            self.assertIn(k, dwb, f"Missing required key {k} in FollowPath")
            self.assertTrue(math.isfinite(dwb.get(k)))

        for k in ['xy_goal_tolerance', 'yaw_goal_tolerance']:
            self.assertIn(k, goal_checker, f"Missing {k} in general_goal_checker")
            self.assertTrue(math.isfinite(goal_checker.get(k)))
            self.assertGreater(goal_checker.get(k), 0.0)

        self.assertGreater(dwb.get('max_vel_x'), 0.0)
        self.assertEqual(dwb.get('min_vel_x'), 0.0)
        self.assertGreater(dwb.get('max_vel_theta'), 0.0)
        self.assertGreater(dwb.get('acc_lim_x'), 0.0)
        self.assertLess(dwb.get('decel_lim_x'), 0.0)
        self.assertEqual(dwb.get('max_vel_y'), 0.0)
        self.assertEqual(dwb.get('min_vel_y'), 0.0)
        self.assertEqual(dwb.get('acc_lim_y'), 0.0)
        self.assertEqual(dwb.get('decel_lim_y'), 0.0)
        self.assertGreaterEqual(dwb.get('min_speed_theta'), 0.0)
        self.assertGreaterEqual(dwb.get('trans_stopped_velocity'), 0.0)
        self.assertLess(dwb.get('trans_stopped_velocity'), dwb.get('max_vel_x'))

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

        # N3QA10-2 params envelope
        self.assertGreaterEqual(dwb.get('min_speed_xy'), 0.0)
        self.assertLessEqual(dwb.get('min_speed_xy'), dwb.get('max_speed_xy'))
        self.assertEqual(dwb.get('max_speed_xy'), dwb.get('max_vel_x'))
        self.assertGreaterEqual(dwb.get('min_speed_theta'), 0.0)
        self.assertLessEqual(dwb.get('min_speed_theta'), dwb.get('max_vel_theta'))
        self.assertEqual(max_vel[2], dwb.get('max_vel_theta'))
        self.assertEqual(min_vel[2], -dwb.get('max_vel_theta'))

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

        # Consistency between controller, smoother, behavior
        self.assertEqual(dwb.get('max_vel_x'), max_vel[0])
        self.assertEqual(dwb.get('min_vel_x'), min_vel[0])
        self.assertEqual(dwb.get('max_vel_theta'), max_vel[2])
        self.assertEqual(dwb.get('max_vel_theta'), behaviors.get('max_rotational_vel'))
        self.assertEqual(dwb.get('acc_lim_x'), max_accel[0])
        self.assertEqual(dwb.get('acc_lim_theta'), max_accel[2])
        self.assertEqual(dwb.get('acc_lim_theta'), behaviors.get('rotational_acc_lim'))
        self.assertEqual(dwb.get('decel_lim_x'), max_decel[0])
        self.assertEqual(dwb.get('decel_lim_theta'), max_decel[2])

        # Check plugin validity (must be strings)
        self.assertTrue(isinstance(dwb.get('plugin'), str))

    def test_baseline(self):
        self._validate_params(self.params)

    def test_qa10_mutations(self):
        import copy

        # QA10-2: max_speed_xy violated
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath']['max_speed_xy'] = 0.0
        p['controller_server']['ros__parameters']['FollowPath']['min_speed_xy'] = 1.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # QA10-2: min_speed_theta violated
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath']['min_speed_theta'] = 999.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # QA10-2: oversized negative angular minimum
        p = copy.deepcopy(self.params)
        p['velocity_smoother']['ros__parameters']['min_velocity'][2] = -999.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 1. Negative DWB tolerance (QA9)
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath']['xy_goal_tolerance'] = -0.1
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 2. NaN DWB tolerance (QA9)
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath']['xy_goal_tolerance'] = float('nan')
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

        # 7. threshold > max speed (QA8)
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath']['trans_stopped_velocity'] = 999.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 8. behavior min > max (QA8)
        p = copy.deepcopy(self.params)
        p['behavior_server']['ros__parameters']['min_rotational_vel'] = 999.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 9. negative shared acceleration (QA8)
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath']['acc_lim_x'] = -1.0
        p['velocity_smoother']['ros__parameters']['max_accel'][0] = -1.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 10. x-acceleration mismatch (QA8)
        p = copy.deepcopy(self.params)
        p['controller_server']['ros__parameters']['FollowPath']['acc_lim_x'] = 1.0
        p['velocity_smoother']['ros__parameters']['max_accel'][0] = 2.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

        # 11. invalid smoother angular bounds (QA8)
        p = copy.deepcopy(self.params)
        p['velocity_smoother']['ros__parameters']['min_velocity'][2] = 1.0
        with self.assertRaises(AssertionError):
            self._validate_params(p)

    def test_no_reverse(self):
        behaviors = self.params.get('behavior_server', {}).get('ros__parameters', {})
        plugins = behaviors.get('behavior_plugins', [])
        self.assertNotIn('backup', plugins)
        self.assertNotIn('drive_on_heading', plugins)
        self.assertNotIn('assisted_teleop', plugins)

        bt_nav = self.params.get('bt_navigator', {}).get('ros__parameters', {})
        bt_plugins = bt_nav.get('plugin_lib_names', [])
        self.assertNotIn('nav2_back_up_action_bt_node', bt_plugins)
        self.assertNotIn('nav2_drive_on_heading_bt_node', bt_plugins)
        self.assertNotIn('nav2_assisted_teleop_action_bt_node', bt_plugins)

    def test_bt_xml_no_reverse(self):
        pkg_share = os.path.join(os.path.dirname(__file__), '..')
        bt_files = [
            'navigate_to_pose_no_reverse.xml',
            'navigate_through_poses_no_reverse.xml'
        ]
        for bt_file in bt_files:
            xml_path = os.path.join(pkg_share, 'behavior_trees', bt_file)
            with open(xml_path, 'r') as f:
                content = f.read()
            self.assertNotIn('<BackUp', content)
            self.assertNotIn('<DriveOnHeading', content)
            self.assertNotIn('<AssistedTeleop', content)

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
            'dwb_core::DWBLocalPlanner'
        )

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

    def test_consistency_stopped_threshold(self):
        controller = self.params.get('controller_server', {}).get('ros__parameters', {})
        dwb = controller.get('FollowPath', {})
        trans_stopped = dwb.get('trans_stopped_velocity')

        # rot_stopped_velocity removed because it is not consumed by DWBLocalPlanner
        self.assertNotIn('rot_stopped_velocity', dwb)
        self.assertTrue(isinstance(trans_stopped, float))

        smoother = self.params.get('velocity_smoother', {}).get('ros__parameters', {})
        deadband = smoother.get('deadband_velocity')
        self.assertIsNotNone(deadband)

        # Deadband must be <= stopped threshold to allow the controller to declare goal reached
        self.assertLessEqual(deadband[0], trans_stopped)


if __name__ == '__main__':
    unittest.main()
