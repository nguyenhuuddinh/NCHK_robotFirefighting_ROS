import os
import yaml
import tempfile
import unittest
import importlib.util

pkg_share = os.path.join(os.path.dirname(__file__), '..')
launch_file = os.path.join(pkg_share, 'launch', 'nav2.launch.py')

spec = importlib.util.spec_from_file_location("nav2_launch", launch_file)
nav2_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav2_launch)


class TestNav2Launch(unittest.TestCase):
    def test_preflight_checks(self):
        from launch.launch_context import LaunchContext

        ctx = LaunchContext()
        ctx.launch_configurations['map'] = ''
        ctx.launch_configurations['params_file'] = 'dummy.yaml'
        with self.assertRaisesRegex(RuntimeError, "The 'map' argument is empty but required."):
            nav2_launch.preflight_checks(ctx)

        ctx.launch_configurations['map'] = 'relative_map.yaml'
        with self.assertRaisesRegex(RuntimeError, "The 'map' argument must be an absolute path"):
            nav2_launch.preflight_checks(ctx)

        ctx.launch_configurations['map'] = '/tmp/does_not_exist_map_12345.yaml'
        with self.assertRaisesRegex(RuntimeError, "The map YAML file does not exist"):
            nav2_launch.preflight_checks(ctx)

        with tempfile.TemporaryDirectory() as tmpdir:
            img_path = os.path.join(tmpdir, 'map.pgm')
            with open(img_path, 'w') as f:
                f.write('P5\n10 10\n255\n' + '0' * 100)

            yaml_path = os.path.join(tmpdir, 'map.yaml')
            with open(yaml_path, 'w') as f:
                yaml.dump({'image': img_path, 'resolution': 0.05, 'origin': [0.0, 0.0, 0.0]}, f)

            # Test map as directory
            ctx.launch_configurations['map'] = tmpdir
            ctx.launch_configurations['params_file'] = 'dummy.yaml'
            with self.assertRaisesRegex(
                    RuntimeError, "The map YAML file does not exist or is not a file"):
                nav2_launch.preflight_checks(ctx)

            # Test malformed map yaml
            malformed_map = os.path.join(tmpdir, 'malformed_map.yaml')
            with open(malformed_map, 'w') as f:
                f.write('image: [1, 2')
            ctx.launch_configurations['map'] = malformed_map
            with self.assertRaisesRegex(RuntimeError, "Failed to parse or validate map YAML"):
                nav2_launch.preflight_checks(ctx)

            # Test non-mapping map yaml
            nonmapping_map = os.path.join(tmpdir, 'nonmapping_map.yaml')
            with open(nonmapping_map, 'w') as f:
                f.write('- item1\n- item2')
            ctx.launch_configurations['map'] = nonmapping_map
            with self.assertRaisesRegex(RuntimeError, "valid mapping"):
                nav2_launch.preflight_checks(ctx)

            # Test missing image in map yaml
            missing_image_map = os.path.join(tmpdir, 'missing_image_map.yaml')
            with open(missing_image_map, 'w') as f:
                yaml.dump({'resolution': 0.05, 'origin': [0.0, 0.0, 0.0]}, f)
            ctx.launch_configurations['map'] = missing_image_map
            with self.assertRaisesRegex(RuntimeError, "missing 'image' key"):
                nav2_launch.preflight_checks(ctx)

            # Test image as directory
            dir_image_map = os.path.join(tmpdir, 'dir_image_map.yaml')
            image_dir = os.path.join(tmpdir, 'image_dir.pgm')
            os.mkdir(image_dir)
            with open(dir_image_map, 'w') as f:
                yaml.dump({'image': image_dir, 'resolution': 0.05, 'origin': [0.0, 0.0, 0.0]}, f)
            ctx.launch_configurations['map'] = dir_image_map
            with self.assertRaisesRegex(
                    RuntimeError, "The map image file does not exist or is not a file"):
                nav2_launch.preflight_checks(ctx)

            # Test params missing
            ctx.launch_configurations['map'] = yaml_path
            ctx.launch_configurations['params_file'] = ''
            with self.assertRaisesRegex(
                    RuntimeError, "The 'params_file' argument is empty but required."):
                nav2_launch.preflight_checks(ctx)

            ctx.launch_configurations['params_file'] = 'relative_params.yaml'
            with self.assertRaisesRegex(
                    RuntimeError, "The 'params_file' argument must be an absolute path"):
                nav2_launch.preflight_checks(ctx)

            ctx.launch_configurations['params_file'] = tmpdir
            with self.assertRaisesRegex(
                    RuntimeError, "The params_file does not exist or is not a file"):
                nav2_launch.preflight_checks(ctx)

            empty_path = os.path.join(tmpdir, 'empty.yaml')
            with open(empty_path, 'w') as f:
                f.write('')
            ctx.launch_configurations['params_file'] = empty_path
            with self.assertRaisesRegex(RuntimeError, "The params_file is empty"):
                nav2_launch.preflight_checks(ctx)

            malformed_path = os.path.join(tmpdir, 'malformed.yaml')
            with open(malformed_path, 'w') as f:
                f.write('foo: [1, 2')
            ctx.launch_configurations['params_file'] = malformed_path
            with self.assertRaisesRegex(RuntimeError, "The params_file is malformed YAML"):
                nav2_launch.preflight_checks(ctx)

            nonmapping_path = os.path.join(tmpdir, 'nonmapping.yaml')
            with open(nonmapping_path, 'w') as f:
                f.write('- item1\n- item2')
            ctx.launch_configurations['params_file'] = nonmapping_path
            with self.assertRaisesRegex(
                    RuntimeError, "The params_file does not contain a valid mapping"):
                nav2_launch.preflight_checks(ctx)

            params_path = os.path.join(tmpdir, 'params.yaml')
            with open(params_path, 'w') as f:
                f.write('dummy: True')

            ctx.launch_configurations['params_file'] = params_path

            # Should pass without exceptions and return None
            self.assertIsNone(nav2_launch.preflight_checks(ctx))

    def test_launch_description(self):
        ld = nav2_launch.generate_launch_description()
        self.assertIsNotNone(ld)

        from launch.actions import DeclareLaunchArgument
        declare_args = {
            a.name: a for a in ld.entities if isinstance(a, DeclareLaunchArgument)
        }

        self.assertIn('use_sim_time', declare_args)
        self.assertEqual(declare_args['use_sim_time'].default_value[0].text, 'false')

        self.assertIn('map', declare_args)
        self.assertIsNone(
            declare_args['map'].default_value,
            "map argument should not have a default value")

        self.assertIn('params_file', declare_args)
        self.assertTrue(
            declare_args['params_file'].default_value[0].text.endswith('nav2_params.yaml'))

        self.assertIn('autostart', declare_args)
        self.assertEqual(declare_args['autostart'].default_value[0].text, 'true')

        self.assertIn('rviz', declare_args)
        self.assertEqual(declare_args['rviz'].default_value[0].text, 'false')
        from launch import LaunchDescription
        self.assertIsInstance(ld, LaunchDescription)
        self._validate_launch(ld)

    def _validate_launch(self, ld):
        from launch.actions import (
            ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
        )
        from launch_ros.actions import Node
        from launch.conditions import IfCondition
        from launch.substitutions import LaunchConfiguration
        import os
        from ament_index_python.packages import get_package_share_directory

        def canonical_text(obj):
            if hasattr(obj, 'text'):
                return str(obj.text)
            elif isinstance(obj, (list, tuple)):
                return "".join([canonical_text(x) for x in obj])
            return str(obj)

        def walk_entities(entities, seen=None):
            if seen is None:
                seen = set()
            for e in entities:
                e_id = id(e)
                if e_id in seen:
                    continue
                seen.add(e_id)
                yield e
                if hasattr(e, 'get_sub_entities'):
                    try:
                        yield from walk_entities(e.get_sub_entities(), seen)
                    except Exception:
                        pass
                if hasattr(e, 'actions'):
                    yield from walk_entities(e.actions, seen)
                if hasattr(e, 'body'):
                    yield from walk_entities(e.body, seen)

        from collections import Counter

        all_entities = list(walk_entities(ld.entities))
        type_counter = Counter(type(e) for e in all_entities)

        expected_action_counts = {
            DeclareLaunchArgument: 5,
            OpaqueFunction: 1,
            ExecuteProcess: 1,
            Node: 10,
        }
        self.assertEqual(
            type_counter, Counter(expected_action_counts),
            f"Exact entity type counter mismatch: expected {expected_action_counts}, "
            f"got {dict(type_counter)}"
        )

        nodes = [e for e in all_entities if type(e) is Node]

        def extract_text(obj):
            if hasattr(obj, 'text'):
                return obj.text
            elif isinstance(obj, (list, tuple)):
                return [extract_text(x) for x in obj]
            elif isinstance(obj, dict):
                return {
                    (tuple(extract_text(x) for x in k) if isinstance(k, (list, tuple))
                     else extract_text(k)): extract_text(v)
                    for k, v in obj.items()
                }
            return obj

        manager_names = []
        for node in nodes:
            name = node.node_executable
            if isinstance(name, list) and len(name) > 0 and hasattr(name[0], 'text'):
                name = name[0].text
            if name == 'lifecycle_manager':
                node_name = getattr(node, '_Node__node_name', '')
                if (isinstance(node_name, list) and len(node_name) > 0 and
                   hasattr(node_name[0], 'text')):
                    node_name = node_name[0].text
                elif not isinstance(node_name, str):
                    node_name = str(node_name)
                manager_names.append(node_name)

                params = getattr(node, 'parameters', getattr(node, '_Node__parameters', []))
                for p in params:
                    p_text = extract_text(p)
                    if isinstance(p_text, dict):
                        for k, v in p_text.items():
                            if isinstance(k, (list, tuple)) and 'node_names' in k:
                                v_flat = []
                                for item in (v if isinstance(v, (list, tuple)) else [v]):
                                    if isinstance(item, (list, tuple)):
                                        for x in item:
                                            s_val = str(x).split('\n')[0]
                                            v_flat.append(s_val.strip(" \n\r\t'\""))
                                    else:
                                        s_val = str(item).split('\n')[0]
                                        v_flat.append(s_val.strip(" \n\r\t'\""))
                                if 'amcl' in v_flat:
                                    self.assertEqual(v_flat, ['map_server', 'amcl'])
                                    param_keys = []
                                    for p_dict in params:
                                        p_text = extract_text(p_dict)
                                        if isinstance(p_text, dict):
                                            param_keys.extend([str(k) for k in p_text.keys()])

                                        # Assert values are LaunchConfiguration
                                        for k, v in p_dict.items():
                                            k_txt = extract_text(k)
                                            if isinstance(k_txt, str) and (
                                                k_txt == 'autostart' or k_txt == 'use_sim_time'
                                            ):
                                                v_item = (
                                                    v[0] if isinstance(v, (list, tuple)) else v
                                                )
                                                self.assertIsInstance(
                                                    v_item, LaunchConfiguration,
                                                    f"{k_txt} must be a LaunchConfiguration")
                                                self.assertEqual(
                                                    canonical_text(v_item.variable_name), k_txt,
                                                    f"{k_txt} must use LaunchConfiguration"
                                                    f"('{k_txt}')")

                                    self.assertTrue(any('autostart' in k for k in param_keys))
                                    self.assertTrue(
                                        any('use_sim_time' in k for k in param_keys))
                                elif 'planner_server' in v_flat:
                                    self.assertEqual(v_flat, [
                                        'controller_server', 'smoother_server',
                                        'planner_server', 'behavior_server',
                                        'bt_navigator', 'velocity_smoother'
                                    ])
                                    param_keys = []
                                    for p_dict in params:
                                        p_text = extract_text(p_dict)
                                        if isinstance(p_text, dict):
                                            param_keys.extend([str(k) for k in p_text.keys()])

                                        # Assert values are LaunchConfiguration
                                        for k, v in p_dict.items():
                                            k_txt = extract_text(k)
                                            if isinstance(k_txt, str) and (
                                                k_txt == 'autostart' or k_txt == 'use_sim_time'
                                            ):
                                                v_item = (
                                                    v[0] if isinstance(v, (list, tuple)) else v
                                                )
                                                self.assertIsInstance(
                                                    v_item, LaunchConfiguration,
                                                    f"{k_txt} must be a LaunchConfiguration")
                                                self.assertEqual(
                                                    canonical_text(v_item.variable_name), k_txt,
                                                    f"{k_txt} must use LaunchConfiguration"
                                                    f"('{k_txt}')")

                                    self.assertTrue(any('autostart' in k for k in param_keys))
                                    self.assertTrue(
                                        any('use_sim_time' in k for k in param_keys))

        self.assertIn('lifecycle_manager_localization', manager_names)
        self.assertIn('lifecycle_manager_navigation', manager_names)

        from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
        from launch_ros.actions import Node
        declares = [e for e in all_entities if type(e) is DeclareLaunchArgument]
        opaques = [e for e in all_entities if type(e) is OpaqueFunction]
        exec_procs = [e for e in all_entities if type(e) is ExecuteProcess]
        all_nodes = [e for e in all_entities if type(e) is Node]

        self.assertEqual(
            len(declares), 5,
            f"Exact DeclareLaunchArgument count should be 5, found {len(declares)}")
        self.assertEqual(
            len(opaques), 1,
            f"Exact OpaqueFunction count should be 1, found {len(opaques)}")
        self.assertEqual(
            len(exec_procs), 1,
            f"Exact non-Node ExecuteProcess count should be 1, found {len(exec_procs)}")
        self.assertEqual(
            len(all_nodes), 10,
            f"Exact Node count should be 10, found {len(all_nodes)}")

        # Validate OpaqueFunction identity (preflight checks)
        self.assertIs(
            opaques[0]._OpaqueFunction__function, nav2_launch.preflight_checks,
            "OpaqueFunction callback must be exactly nav2_launch.preflight_checks")
        self.assertEqual(len(opaques[0]._OpaqueFunction__args), 0)
        self.assertEqual(len(opaques[0]._OpaqueFunction__kwargs), 0)

        # Exact Node multiset check: (package, executable, name)
        # Note: name can be obtained via _Node__node_name or node_name if we can extract it.
        # But we must canonical_text everything.
        actual_nodes = []
        for node in all_nodes:
            pkg = canonical_text(getattr(node, '_Node__package', ''))
            exe = canonical_text(getattr(node, 'node_executable', ''))

            raw_name = getattr(node, '_Node__node_name', None)
            if raw_name is None:
                name = exe
            else:
                name = canonical_text(raw_name)
                if name == 'None' or name == 'UNSPECIFIED_NODE_NAME':
                    name = exe

            actual_nodes.append((pkg, exe, name))

        expected_node_multiset = [
            ('nav2_map_server', 'map_server', 'map_server'),
            ('nav2_amcl', 'amcl', 'amcl'),
            ('nav2_controller', 'controller_server', 'controller_server'),
            ('nav2_smoother', 'smoother_server', 'smoother_server'),
            ('nav2_planner', 'planner_server', 'planner_server'),
            ('nav2_behaviors', 'behavior_server', 'behavior_server'),
            ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
            ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother'),
            ('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_localization'),
            ('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation')
        ]

        # Sort and compare multisets exactly
        actual_nodes.sort()
        expected_node_multiset.sort()
        self.assertEqual(actual_nodes, expected_node_multiset, "Exact Node multiset mismatch")

        # No IncludeLaunchDescription allowed anywhere
        include_procs = [e for e in all_entities if type(e) is IncludeLaunchDescription]
        self.assertEqual(
            len(include_procs), 0, "No IncludeLaunchDescription allowed in this launch file")

        rviz_proc = exec_procs[0]
        cmd_text = [canonical_text(c) for c in rviz_proc.cmd]

        # exact rviz2 -d <nav2_default_view.rviz>
        expected_rviz_path = os.path.join(
            get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
        self.assertEqual(len(cmd_text), 3, "RViz command must have exactly 3 arguments")
        self.assertEqual(cmd_text[0], 'rviz2')
        self.assertEqual(cmd_text[1], '-d')
        self.assertEqual(cmd_text[2], expected_rviz_path)

        # exact RViz condition phải dùng LaunchConfiguration rviz
        self.assertIsInstance(rviz_proc.condition, IfCondition)
        pred_expr = rviz_proc.condition._IfCondition__predicate_expression
        lc = pred_expr[0] if isinstance(pred_expr, list) else pred_expr
        self.assertIsInstance(lc, LaunchConfiguration)
        self.assertEqual(canonical_text(lc.variable_name), 'rviz')

    def test_qa9_mutations(self):
        from launch.actions import (
            ExecuteProcess, GroupAction, SetLaunchConfiguration, OpaqueFunction
        )
        from launch.conditions import IfCondition
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import Node

        def walk_entities(entities, seen=None):
            if seen is None:
                seen = set()
            for e in entities:
                e_id = id(e)
                if e_id in seen:
                    continue
                seen.add(e_id)
                yield e
                if hasattr(e, 'get_sub_entities'):
                    try:
                        yield from walk_entities(e.get_sub_entities(), seen)
                    except Exception:
                        pass
                if hasattr(e, 'actions'):
                    yield from walk_entities(e.actions, seen)
                if hasattr(e, 'body'):
                    yield from walk_entities(e.body, seen)

        # QA10-1: Opaque callback spoofing __name__ (replacement duy nhất)
        def preflight_checks(context, *args, **kwargs):
            return [Node(package='fire_robot_bringup', executable='serial_bridge_node')]

        ld_spoof = nav2_launch.generate_launch_description()
        for e in walk_entities(ld_spoof.entities):
            if type(e) is OpaqueFunction:
                e._OpaqueFunction__function = preflight_checks
        with self.assertRaisesRegex(AssertionError, "OpaqueFunction callback must be exactly"):
            self._validate_launch(ld_spoof)

        # QA11: Opaque subclass mutation
        class EvilOpaque(OpaqueFunction):
            pass

        ld_subclass = nav2_launch.generate_launch_description()
        for i, e in enumerate(ld_subclass.entities):
            if type(e) is OpaqueFunction:
                ld_subclass.entities[i] = EvilOpaque(function=nav2_launch.preflight_checks)
        with self.assertRaisesRegex(AssertionError, "Exact entity type counter mismatch"):
            self._validate_launch(ld_subclass)

        # QA12: Append HiddenHardwareOpaque while keeping original Opaque
        class HiddenHardwareOpaque(OpaqueFunction):
            def execute(self, context):
                return [Node(package='fire_robot_bringup', executable='serial_bridge_node')]

        ld_hidden = nav2_launch.generate_launch_description()
        ld_hidden.add_action(HiddenHardwareOpaque(function=nav2_launch.preflight_checks))
        with self.assertRaisesRegex(AssertionError, "Exact entity type counter mismatch"):
            self._validate_launch(ld_hidden)

        # QA13: Append custom LaunchDescriptionEntity
        from launch import LaunchDescriptionEntity

        class EvilEntity(LaunchDescriptionEntity):
            def visit(self, context):
                return [Node(package='fire_robot_bringup', executable='serial_bridge_node')]

        ld_entity = nav2_launch.generate_launch_description()
        ld_entity.add_entity(EvilEntity())
        with self.assertRaisesRegex(AssertionError, "Exact entity type counter mismatch"):
            self._validate_launch(ld_entity)

        # QA9-1: Mutated Node package (controller_server)
        ld_pkg = nav2_launch.generate_launch_description()

        def canonical_text(obj):
            if hasattr(obj, 'text'):
                return str(obj.text)
            elif isinstance(obj, (list, tuple)):
                return "".join([canonical_text(x) for x in obj])
            return str(obj)
        for e in walk_entities(ld_pkg.entities):
            if isinstance(e, Node):
                exe_txt = canonical_text(getattr(e, 'node_executable', ''))
                if exe_txt == 'controller_server':
                    e._Node__package = [type(e.node_executable[0])('wrong_package')]
        with self.assertRaises(AssertionError):
            self._validate_launch(ld_pkg)

        # 1. SetLaunchConfiguration override (QA8)
        ld_override = nav2_launch.generate_launch_description()
        ld_override.add_action(SetLaunchConfiguration('autostart', 'false'))
        with self.assertRaises(AssertionError):
            self._validate_launch(ld_override)

        # 2. Nested hardware node (QA8)
        ld_hw = nav2_launch.generate_launch_description()
        node_hw = Node(package='fire_robot_bringup', executable='serial_bridge_node')
        ld_hw.add_action(GroupAction([node_hw]))
        with self.assertRaises(AssertionError):
            self._validate_launch(ld_hw)

        # 3. Wrong lifecycle name (QA8)
        ld_wrong = nav2_launch.generate_launch_description()
        node_wrong_lifecycle = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            parameters=[{'autostart': LaunchConfiguration('wrong_autostart_gate')}]
        )
        ld_wrong.add_action(node_wrong_lifecycle)
        with self.assertRaises(AssertionError):
            self._validate_launch(ld_wrong)

        # 4. Fake RViz substrings (QA8)
        ld_fake_rviz = nav2_launch.generate_launch_description()
        ep_fake_rviz = ExecuteProcess(
            cmd=['evil_rviz2_wrapper', '-not-d', 'nav2_default_view.rviz.evil'],
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
        ld_fake_rviz.add_action(ep_fake_rviz)
        with self.assertRaises(AssertionError):
            self._validate_launch(ld_fake_rviz)


if __name__ == '__main__':
    unittest.main()
