#!/usr/bin/env python3
"""Static local installation inventory only; never starts ROS, devices or peers.

PASS means that files/metadata match, not that CUDA, DDS, devices or firmware
have passed runtime. Run independently on each host after its local build.
"""
import argparse
import ast
import hashlib
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import struct

ROOT = Path(__file__).resolve().parents[1]
MACHINES = {'x86_64': 62, 'aarch64': 183}
MODEL_SHA = '28d3b25f70ccd0ea958a2abfa11f35804f5a82c2f711e4786d5bfa6cce46baca'
NAV_STATUS_REFRESH_MARKER = b'Nav2 actual action status refresh enabled at 5 Hz'
MAP_SHA = {
    'fire_room_9_10_2026.yaml': '359cbdbb6a6c7e63db0f8800a9c71f95dc69cb409631b0c2f89917197f4d81b2',
    'fire_room_9_10_2026.pgm': '8f77bbed522eee571483237c32c6f3940d7687297a677cd56439658c94cfd510',
}


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def checked_map(root):
    import yaml
    folder = Path(root) / 'ros2_ws/src/fire_robot_navigation/maps'
    for name, expected in MAP_SHA.items():
        if sha(folder / name) != expected:
            raise ValueError('DEFAULT_MAP_CHANGED:' + name)
    row = yaml.safe_load((folder / 'fire_room_9_10_2026.yaml').read_text())
    if row['image'] != 'fire_room_9_10_2026.pgm':
        raise ValueError('DEFAULT_MAP_IMAGE_NOT_LOCAL')
    return {name: sha(folder / name) for name in MAP_SHA}


def checked_elf(path, machine, *, executable=False):
    path = Path(path)
    if executable and not os.access(path, os.X_OK):
        raise ValueError('MISSING_EXECUTABLE_PERMISSION:' + str(path))
    with path.open('rb') as stream:
        header = stream.read(20)
    if len(header) != 20 or header[:6] != b'\x7fELF\x02\x01':
        raise ValueError('EXPECTED_64BIT_LITTLE_ENDIAN_ELF:' + str(path))
    if machine not in MACHINES or struct.unpack('<H', header[18:20])[0] != MACHINES[machine]:
        raise ValueError('ELF_HOST_ARCH_MISMATCH:' + str(path))
    return dict(path=str(path), resolved=str(path.resolve()), sha256=sha(path), machine=machine)


def checked_nav_container(path, machine):
    result=checked_elf(path,machine,executable=True)
    if NAV_STATUS_REFRESH_MARKER not in Path(path).read_bytes():
        raise ValueError('NAV2_STATUS_REFRESH_BUILD_REQUIRED')
    return dict(result,actual_status_refresh_available=True)


def checked_wrapper(path):
    path = Path(path)
    if not path.is_file() or not os.access(path, os.X_OK):
        raise ValueError('MISSING_EXECUTABLE:' + str(path))
    first = path.read_text().splitlines()[0]
    parts = shlex.split(first[2:]) if first.startswith('#!') else []
    # Ubuntu rosbridge uses this standard shebang rather than a generated
    # ament wrapper. Resolve its Python locally without executing env/Python.
    if parts == ['/usr/bin/env', 'python3']:
        resolved = shutil.which('python3')
        interpreter = Path(resolved) if resolved else None
    elif len(parts) == 1:
        interpreter = Path(parts[0])
    else:
        interpreter = None
    if interpreter is None or not interpreter.is_absolute() or not interpreter.is_file() or not os.access(interpreter, os.X_OK):
        raise ValueError('INVALID_WRAPPER_INTERPRETER:' + str(path))
    return dict(path=str(path), shebang=first, interpreter=str(interpreter), sha256=sha(path))


def source_entries(path):
    """Read setuptools declarations without executing setup.py."""
    tree = ast.parse(Path(path).read_text())
    for call in ast.walk(tree):
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == 'setup':
            for arg in call.keywords:
                if arg.arg == 'entry_points':
                    rows = ast.literal_eval(arg.value)['console_scripts']
                    return {
                        row.split('=', 1)[0].strip(): row.split('=', 1)[1].strip() for row in rows}
    raise ValueError('NO_CONSOLE_DECLARATIONS:' + str(path))


class AmentFiles:
    """Lazy imports: no node constructors, launch execution or ROS context."""
    def __init__(self):
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        from ament_index_python.resources import get_resource
        self.prefix = lambda name: Path(get_package_prefix(name))
        self.share = lambda name: Path(get_package_share_directory(name))
        self.resource = get_resource

    def installed_sources(self, root, role):
        from operational_preflight import installed_sources
        return installed_sources(root, role)

    def entries(self, name):
        return {e.name: e.value for e in importlib.metadata.distribution(name).entry_points if e.group == 'console_scripts'}

    def module(self, name):
        return importlib.util.find_spec(name) is not None


def inventory(root, role, *, files=None, machine=None, model=None, yolo_python=None):
    """Failures are explicit; no network/pgrep/clock/launch operations here."""
    if role not in ('pi', 'laptop'):
        raise ValueError('UNKNOWN_HOST_ROLE')
    root = Path(root)
    machine = platform.machine() if machine is None else machine
    result = dict(schema_version=1, role=role, host_machine=machine, live=False,
                  scope='STATIC_FILES_AND_METADATA_ONLY', checks={}, errors=[],
                  runtime_verified=False, Pi_copy_or_build_performed=False)

    def check(name, operation):
        try:
            result['checks'][name] = operation()
        except Exception as exc:
            result['errors'].append(dict(check=name, error=type(exc).__name__ + ':' + str(exc)))

    def packet():
        from operational_entry import packet_digest
        return packet_digest(root)

    check('source_packet_sha256', packet)
    check('default_map', lambda: checked_map(root))
    expected_machine = 'aarch64' if role == 'pi' else 'x86_64'
    if machine != expected_machine:
        result['errors'].append(dict(check='role_architecture', error='EXPECTED_' + expected_machine))
    if files is None:
        try:
            files = AmentFiles()
        except Exception as exc:
            result['errors'].append(dict(check='ament_environment', error=str(exc)))
            result['status'] = 'STATIC_INSTALL_BLOCKED'
            return result

    check('installed_sources', lambda: [dict(path=str(p), sha256=sha(p)) for p in files.installed_sources(root, role)])
    packages = ('fire_robot_mission', 'fire_robot_bringup', 'fire_robot_safety') if role == 'pi' else ('fire_robot_mission', 'fire_robot_perception')
    for package in packages:
        def wrappers(package=package):
            wanted = source_entries(root / 'ros2_ws/src' / package / 'setup.py')
            actual = files.entries(package)
            for name, target in wanted.items():
                if actual.get(name) != target:
                    raise ValueError('INSTALLED_ENTRYPOINT_NOT_CANDIDATE:' + name)
            return [checked_wrapper(files.prefix(package) / 'lib' / package / name) for name in wanted]
        check(package + '_console_scripts', wrappers)

    # URDF/BT assets are used by included launches but were outside the earlier
    # installed_sources launch/config/module inventory.
    assets = ('fire_robot_description', 'urdf') if role == 'pi' else ('fire_robot_navigation', 'behavior_trees')
    def installed_assets():
        package, folder = assets
        source = root / 'ros2_ws/src' / package / folder
        rows = []
        for path in sorted(source.rglob('*')):
            if path.is_file():
                installed = files.share(package) / folder / path.relative_to(source)
                if sha(installed) != sha(path):
                    raise ValueError('INSTALLED_ASSET_NOT_CANDIDATE:' + str(path))
                rows.append(dict(path=str(installed), sha256=sha(path)))
        if not rows:
            raise ValueError('NO_REQUIRED_ASSETS:' + folder)
        return rows
    check('included_assets', installed_assets)
    dependencies = ['rclpy', 'nav2_msgs', 'lifecycle_msgs', 'tf2_ros', 'cv2', 'numpy', 'yaml']
    dependencies += ['serial'] if role == 'pi' else []
    def python_dependencies():
        missing = [name for name in dependencies if not files.module(name)]
        if missing:
            raise ValueError('PYTHON_MODULES_MISSING:' + ','.join(missing))
        return dependencies
    check('python_module_presence', python_dependencies)

    if role == 'pi':
        for package, executable in (('camsense_x1', 'camsense_x1_node'), ('robot_state_publisher', 'robot_state_publisher')):
            check(executable, lambda package=package, executable=executable: checked_elf(files.prefix(package) / 'lib' / package / executable, machine, executable=True))
        check('rosbridge_websocket', lambda: checked_wrapper(files.prefix('rosbridge_server') / 'lib/rosbridge_server/rosbridge_websocket'))
        check('xacro', lambda: checked_wrapper(files.prefix('xacro') / 'lib/xacro/xacro'))
    else:
        check('nav2_container', lambda: checked_nav_container(files.prefix('fire_robot_navigation') / 'lib/fire_robot_navigation/fire_robot_component_container_isolated', machine))
        def components():
            launch_path = root / 'ros2_ws/src/fire_robot_navigation/launch/nav2_composed_shadow.launch.py'
            tree = ast.parse(launch_path.read_text())
            specs = next(ast.literal_eval(node.value) for node in tree.body if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'COMPONENTS' for t in node.targets))
            rows = []
            for _, package, plugin in specs:
                content, prefix = files.resource('rclcpp_components', package)
                registered = dict(line.split(';', 1) for line in content.splitlines() if ';' in line)
                if plugin not in registered:
                    raise ValueError('MISSING_NAV2_COMPONENT:' + package + '/' + plugin)
                rows.append(dict(plugin=plugin, **checked_elf(Path(prefix) / registered[plugin], machine)))
            return rows
        check('nav2_component_libraries', components)
        check('rviz2', lambda: checked_elf(files.prefix('rviz2') / 'lib/rviz2/rviz2', machine, executable=True))
        from_model = model or Path('/media/huudinh/New Volume/esp32/Fire_smoke_person_detection/best_29_10_2026.pt')
        from_python = yolo_python or Path('/home/huudinh/.venvs/fire_robot_yolo/bin/python')
        def model_file():
            actual = sha(from_model)
            if actual != MODEL_SHA:
                raise ValueError('YOLO_MODEL_NOT_PINNED')
            return dict(path=str(from_model), sha256=actual)
        def yolo_environment():
            # Read venv metadata, do not import torch/model or probe CUDA.
            interpreter = Path(from_python)
            if not interpreter.is_file() or not os.access(interpreter, os.X_OK):
                raise ValueError('YOLO_PYTHON_MISSING')
            versions = {}
            for name in ('torch', 'ultralytics'):
                rows = list((interpreter.parent.parent / 'lib').glob('python*/site-packages/' + name + '-*.dist-info/METADATA'))
                if len(rows) != 1:
                    raise ValueError('YOLO_VENV_METADATA_MISSING_OR_AMBIGUOUS:' + name)
                versions[name] = next(line[9:] for line in rows[0].read_text().splitlines() if line.startswith('Version: '))
            return dict(interpreter=str(interpreter), versions=versions, CUDA_and_model_load='NOT_TESTED')
        check('yolo_model', model_file)
        check('yolo_venv_metadata', yolo_environment)
    result['status'] = 'STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED' if not result['errors'] else 'STATIC_INSTALL_BLOCKED'
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=['pi', 'laptop'], required=True)
    args = parser.parse_args(argv)
    row = inventory(ROOT, args.role)
    print(json.dumps(row, indent=2, ensure_ascii=False))
    return 0 if not row['errors'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
