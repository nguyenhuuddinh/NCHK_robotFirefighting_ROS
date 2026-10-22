"""Camera-child transport selection; never modifies the launch parent's environment."""
from pathlib import Path


def camera_process_environment(publication_mode, transport, profile_path, inherited_env):
    """Return explicit child overrides, rejecting conflicting opt-in configurations.

    Nonblocking UDP may lose camera datagrams when the socket is full. It does
    not guarantee a bounded DDS publish call and is not appropriate as a global
    override for control traffic. ROS QoS and camera freshness checks stay intact.
    """
    if publication_mode not in ('ASYNCHRONOUS', 'SYNCHRONOUS'):
        raise ValueError('Unsupported native camera publication mode')
    env = {'RMW_FASTRTPS_PUBLICATION_MODE': publication_mode}
    if transport == 'system':
        return env
    if transport != 'wifi_nonblocking':
        raise ValueError('Unsupported native camera transport')
    rmw = inherited_env.get('RMW_IMPLEMENTATION', '')
    if rmw and rmw != 'rmw_fastrtps_cpp':
        raise ValueError('wifi_nonblocking requires rmw_fastrtps_cpp; use system for another RMW')
    for variable in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
        if inherited_env.get(variable):
            raise ValueError('wifi_nonblocking conflicts with inherited ' + variable + '; use system or review that profile')
    if inherited_env.get('RMW_FASTRTPS_USE_QOS_FROM_XML', '0') not in ('', '0'):
        raise ValueError('wifi_nonblocking requires ROS QoS; review RMW_FASTRTPS_USE_QOS_FROM_XML')
    profile = Path(profile_path)
    if not profile.is_file():
        raise FileNotFoundError('Missing native camera DDS profile: ' + str(profile))
    env.update(RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
               FASTRTPS_DEFAULT_PROFILES_FILE=str(profile.resolve()))
    return env
