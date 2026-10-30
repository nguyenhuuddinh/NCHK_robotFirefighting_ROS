"""Explicit localhost-only real Nav2 teardown experiment, no hardware endpoints."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--ordered', action='store_true')
    parser.add_argument('--wrapper', action='store_true')
    parser.add_argument('--early', action='store_true')
    parser.add_argument('--goal', action='store_true')
    parser.add_argument('--goal-active', action='store_true')
    parser.add_argument('--health', action='store_true')
    parser.add_argument('--pause-owned', type=float, default=0.)
    parser.add_argument('--stop-when', choices=('delay', 'reset', 'recovered'), default='delay')
    parser.add_argument('--resume-delay', type=float, default=2.)
    parser.add_argument('--repeat-q', type=int, default=1)
    parser.add_argument('--trace-native', action='store_true', help='QA-only debugger on the owned container')
    parser.add_argument('--legacy-app', action='store_true', help='Probe the sealed e2 app via PYTHONPATH without new diagnostics')
    args = parser.parse_args()
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') == '216'
    assert 0 <= args.pause_owned <= 8.
    assert 0 <= args.resume_delay <= 5. and 1 <= args.repeat_q <= 5
    assert args.stop_when == 'delay' or args.pause_owned > 4.
    args.out.mkdir(parents=True, exist_ok=False)
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.srv import ManageLifecycleNodes
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from tf2_ros import StaticTransformBroadcaster
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('desktop_nav_shutdown_qa')
    tf = StaticTransformBroadcaster(node)
    transforms = []
    for parent, child in [('map', 'odom'), ('odom', 'base_link'), ('base_link', 'laser_frame')]:
        msg = TransformStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.rotation.w = 1.
        transforms.append(msg)
    tf.sendTransform(transforms)
    scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
    def inputs():
        scan = LaserScan()
        scan.header.stamp = node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min, scan.angle_max, scan.angle_increment = -3.14, 3.14, 6.28 / 359
        scan.range_min, scan.range_max = .1, 8.
        scan.ranges = [2.] * 360
        scan_pub.publish(scan)
        odom = Odometry()
        odom.header.stamp = scan.header.stamp
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
    timer = node.create_timer(.1, inputs)
    stop_thread = threading.Event()
    def spin():
        while not stop_thread.is_set():
            rclpy.spin_once(node, timeout_sec=.05)
    thread = threading.Thread(target=spin)
    thread.start()
    root = Path(__file__).resolve().parents[3]
    cmd = ['ros2', 'launch', '--noninteractive', 'fire_robot_navigation', 'nav2_composed_shadow.launch.py',
           'map:=' + str(root / 'ros2_ws/src/fire_robot_navigation/maps/TN1_30_10_2026.yaml'),
           'params_file:=' + str(root / 'releases/r5_20261029/ros/ros2_ws/src/fire_robot_navigation/config/nav2_params.yaml'),
           'rviz:=false', 'refresh_bt_action_status:=true']
    if args.trace_native:
        import shlex
        cmd[3:3] = ['--launch-prefix', shlex.join([sys.executable, '-B', str(Path(__file__).with_name('nav_trace_exec.py').resolve())]),
                    '--launch-prefix-filter', '.*fire_robot_component_container_isolated.*']
    if args.wrapper:
        options = [] if args.legacy_app else ['--diagnostics', str(args.out / 'nav_performance.jsonl')]
        cmd += ['sigterm_timeout:=8.0', 'sigkill_timeout:=3.0']
        if args.trace_native:
            # ROS CLI prefix flags must precede the package. Use the existing
            # explicit test injection API, keeping the deployed CLI guard intact.
            driver = ('import sys; from services.nav_session import supervise; '
                      'raise SystemExit(supervise(sys.argv[2:], diagnostic_path=sys.argv[1], '
                      'rpc_command=[sys.executable,"-m","services.nav_session","--rpc"]))')
            cmd = [sys.executable, '-B', '-c', driver, str(args.out/'nav_performance.jsonl'), *cmd]
        else:
            cmd = [sys.executable, '-m', 'services.nav_session', *options, '--', *cmd]
    log = (args.out / 'nav.log').open('wb')
    child = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    health = health_log = debugger = debugger_log = None
    if args.health:
        health_log = (args.out / 'health_worker.log').open('wb')
        entry = ['services.app_health_entry'] if args.legacy_app else ['services.fire_health_entry', '--packet', 'a'*64]
        health = subprocess.Popen([sys.executable, '-m', *entry,
            '--out', str(args.out/'health.json'), '--session', '8518a120-a012-4db3-bbf3-f73f67aed61e',
            '--role', 'laptop'], stdout=health_log, stderr=subprocess.STDOUT, start_new_session=True)
    result = {'ordered': args.ordered, 'wrapper': args.wrapper, 'early': args.early,
              'scope': 'ROS_LOCALHOST_216_SYNTHETIC_INPUTS_NO_ROBOT', 'pid': child.pid,
              'stop_when': args.stop_when, 'repeat_q': args.repeat_q, 'events': []}
    def event(name, **fields):
        result['events'].append(dict(event=name, mono=time.monotonic(), wall_ns=time.time_ns(), **fields))
    event('LAUNCH_STARTED')
    try:
        if not args.early:
            ready = node.create_client(GetState, '/bt_navigator/get_state')
            deadline = time.monotonic() + 35.
            active = False
            while time.monotonic() < deadline and child.poll() is None:
                if ready.service_is_ready():
                    future = ready.call_async(GetState.Request())
                    at = time.monotonic()
                    while not future.done() and time.monotonic() - at < 1.:
                        time.sleep(.02)
                    if future.done() and future.result().current_state.id == 3:
                        active = True
                        break
                time.sleep(.1)
            result['active_before_stop'] = active
            if not active:
                raise RuntimeError('Nav2 did not reach ACTIVE')
            if args.goal or args.goal_active:
                from nav2_msgs.action import NavigateToPose
                from rclpy.action import ActionClient
                client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
                assert client.wait_for_server(timeout_sec=3.)
                request = NavigateToPose.Goal()
                request.pose.header.frame_id = 'map'
                request.pose.header.stamp = node.get_clock().now().to_msg()
                request.pose.pose.orientation.w = 1.
                request.pose.pose.position.x = .5 if args.goal_active else 0.
                future = client.send_goal_async(request)
                until = time.monotonic() + 4.
                while not future.done() and time.monotonic() < until:
                    time.sleep(.02)
                assert future.done() and future.result().accepted
                handle = future.result()
                done = handle.get_result_async()
                until = time.monotonic() + (1. if args.goal_active else 7.)
                while not done.done() and time.monotonic() < until:
                    time.sleep(.02)
                if not done.done() and not args.goal_active:
                    handle.cancel_goal_async()
                    time.sleep(.5)
                result['goal_terminal'] = done.done()
                result['goal_status'] = done.result().status if done.done() else None
            time.sleep(1.)
            if args.trace_native:
                import re
                labels = re.findall(r'process started with pid \[(\d+)\]', (args.out/'nav.log').read_text())
                assert len(labels) == 1
                inferior = int(labels[0])
                assert os.getpgid(inferior) == child.pid
                assert Path(os.readlink('/proc/'+str(inferior)+'/exe')).name == 'fire_robot_component_container_isolated'
                debugger_log = (args.out/'gdb.log').open('wb')
                debugger = subprocess.Popen(['gdb', '-q', '-batch',
                    '-iex', 'set auto-solib-add off', '-iex', 'set debuginfod enabled off',
                    '-p', str(inferior), '-ex', 'set pagination off',
                    '-ex', 'handle SIGINT nostop noprint pass', '-ex', 'handle SIGSTOP nostop noprint pass',
                    '-ex', 'handle SIGCONT nostop noprint pass', '-ex', 'echo APP_QA_GDB_READY\\n',
                    '-ex', 'shell /bin/sleep '+str(args.pause_owned), '-ex', 'echo APP_QA_GDB_RESUMING\\n',
                    '-ex', 'continue', '-ex', 'sharedlibrary', '-ex', 'thread apply all bt 24',
                    '-ex', 'info registers rip', '-ex', 'info sharedlibrary', '-ex', 'quit'],
                    stdin=subprocess.DEVNULL, stdout=debugger_log, stderr=subprocess.STDOUT, start_new_session=True)
                until = time.monotonic()+15.
                while time.monotonic() < until:
                    if 'APP_QA_GDB_READY' in (args.out/'gdb.log').read_text():
                        event('QA_DEBUGGER_ATTACHED', pid=inferior)
                        break
                    assert debugger.poll() is None, (args.out/'gdb.log').read_text()
                    time.sleep(.02)
                else:
                    raise RuntimeError('QA debugger attach timed out')
            if args.pause_owned:
                import re
                labels = re.findall(r'process started with pid \[(\d+)\]', (args.out/'nav.log').read_text())
                assert len(labels) == 1
                container = int(labels[0])
                assert os.getpgid(container) == child.pid
                offset = len((args.out/'nav.log').read_text())
                if args.trace_native:
                    # GDB intercepts/continues SIGSTOP. Instead hold its attached
                    # inferior for the same bounded interval before `continue`.
                    event('OWNED_CONTAINER_PAUSED', pid=container, method='QA_GDB_HOLD')
                    until = time.monotonic()+args.pause_owned+3.
                    while 'APP_QA_GDB_RESUMING' not in (args.out/'gdb.log').read_text():
                        assert time.monotonic() < until, 'QA debugger hold timed out'
                        time.sleep(.01)
                    event('OWNED_CONTAINER_RESUMED', pid=container, method='QA_GDB_CONTINUE')
                else:
                    try:
                        os.kill(container, signal.SIGSTOP)
                        event('OWNED_CONTAINER_PAUSED', pid=container)
                        time.sleep(args.pause_owned)
                    finally:
                        os.kill(container, signal.SIGCONT)
                        event('OWNED_CONTAINER_RESUMED', pid=container)
                if args.stop_when == 'delay':
                    time.sleep(args.resume_delay)
                else:
                    until = time.monotonic() + 40.
                    while time.monotonic() < until and child.poll() is None:
                        tail = (args.out/'nav.log').read_text()[offset:]
                        reset = 'Resetting managed nodes...' in tail
                        recovered = all(any('['+name+']' in line and 'Managed nodes are active' in line
                                            for line in tail.splitlines()) for name in
                                        ('lifecycle_manager_navigation', 'lifecycle_manager_localization'))
                        if reset and (args.stop_when == 'reset' or recovered):
                            event('STOP_PHASE_OBSERVED', phase=args.stop_when)
                            break
                        time.sleep(.01)
                    else:
                        raise RuntimeError('Requested recovery stop phase was not observed')
                result['injected_owned_pause_s'] = args.pause_owned
                result['bond_reset_observed_after_pause'] = 'CRITICAL FAILURE:' in (args.out/'nav.log').read_text()
        else:
            time.sleep(.3)
        start = time.monotonic()
        result['log_tail_before_q'] = (args.out/'nav.log').read_text().splitlines()[-12:]
        event('Q_REQUESTED')
        if health is not None and health.poll() is None:
            # R5 StackCleanup stops startup_health in the same stage as nav.
            # Do not keep an extra service observer alive through teardown.
            health.send_signal(signal.SIGTERM)
            result['health_stop_requested_with_nav'] = True
        if args.ordered:
            result['lifecycle'] = []
            for name in ('lifecycle_manager_navigation', 'lifecycle_manager_localization'):
                client = node.create_client(ManageLifecycleNodes, '/' + name + '/manage_nodes')
                assert client.wait_for_service(timeout_sec=2.)
                request = ManageLifecycleNodes.Request()
                request.command = request.SHUTDOWN
                future = client.call_async(request)
                until = time.monotonic() + 10.
                while not future.done() and time.monotonic() < until:
                    time.sleep(.02)
                success = future.done() and future.result().success
                result['lifecycle'].append({'node': name, 'success': success})
                assert success
        child.send_signal(signal.SIGINT)
        for index in range(args.repeat_q - 1):
            time.sleep(.03)
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                event('Q_REPEATED', index=index+1)
        result['rc'] = child.wait(timeout=25.)
        result['elapsed_s'] = time.monotonic() - start
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        if debugger is not None:
            try:
                result['debugger_rc'] = debugger.wait(timeout=20.)
            except subprocess.TimeoutExpired:
                debugger.terminate()
                debugger.wait(timeout=3.)
                result['debugger_timed_out'] = True
            debugger_log.close()
        if health is not None:
            if health.poll() is None:
                health.send_signal(signal.SIGTERM)
            try:
                result['health_worker_rc'] = health.wait(timeout=4.)
            except subprocess.TimeoutExpired:
                health.kill()
                health.wait(timeout=2.)
                result['error'] = 'HEALTH_WORKER_FORCED'
            health_log.close()
        sys.path.insert(0, str(root / 'releases/r5_20261029/ros/scripts'))
        from operational_support.cleanup_group import inspect
        _, live = inspect(child.pid)
        result['live_before_test_cleanup'] = live
        if child.poll() is not None and live:
            result['error'] = 'DESCENDANTS_REMAINED_AFTER_LAUNCH_EXIT'
        try:
            os.killpg(child.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        if child.poll() is None:
            try:
                child.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=3.)
        stop_thread.set()
        thread.join()
        node.destroy_node()
        rclpy.shutdown()
        log.close()
    sys.path.insert(0, str(root / 'releases/r5_20261029/ros/scripts'))
    from operational_logs import audit_launch_children
    result['child_audit'] = audit_launch_children((args.out / 'nav.log').read_text())
    result['lifecycle_helpers_started'] = (args.out/'nav.log').read_text().count('"event": "LIFECYCLE_BEGIN"')
    if not result['child_audit']['clean']:
        result['error'] = 'NAV_CHILD_AUDIT_UNCLEAN'
    if args.wrapper and not args.legacy_app:
        performance = [json.loads(l) for l in (args.out/'nav_performance.jsonl').read_text().splitlines()]
        result['performance_samples'] = len(performance)
        result['performance_has_owned_processes'] = any(len(r['processes']) >= 2 for r in performance)
    if args.health:
        detail = [json.loads(l) for p in args.out.glob('health_diag_*.jsonl') for l in p.read_text().splitlines()]
        result['health_diagnostic_samples'] = len(detail)
        result['localization_blockers_logged'] = bool(detail) and all('localization_blockers' in r for r in detail)
    (args.out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return int('error' in result)


if __name__ == '__main__':
    raise SystemExit(main())
