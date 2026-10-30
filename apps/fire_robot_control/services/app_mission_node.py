"""Process-local app overrides for two existing mission entrypoints only."""
import runpy
import sys
from pathlib import Path


def install(setattr_fn=setattr):
    import fire_robot_mission.live_nav_stop_gateway as gateway
    import fire_robot_mission.cycle_endpoint as endpoint
    import fire_robot_mission.nav2_cancel_guard as guard
    from .app_task_recovery import AppCycleEndpoint, AppNavCore
    setattr_fn(endpoint, 'CycleEndpoint', AppCycleEndpoint)
    setattr_fn(gateway, 'CycleEndpoint', AppCycleEndpoint)
    setattr_fn(gateway, 'LiveNavStopCore', AppNavCore)
    setattr_fn(guard, 'CycleEndpoint', AppCycleEndpoint)


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if args and args[0] == '--':
        args.pop(0)
    if not args or Path(args[0]).name not in ('live_nav_stop_gateway', 'nav2_cancel_guard'):
        raise ValueError('APP_MISSION_WRAPPER_ROLE_INVALID')
    install()
    sys.argv = args
    runpy.run_path(args[0], run_name='__main__')


if __name__ == '__main__':
    main()
