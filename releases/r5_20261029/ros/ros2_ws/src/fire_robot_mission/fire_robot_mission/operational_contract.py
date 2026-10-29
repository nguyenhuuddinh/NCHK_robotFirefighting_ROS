"""Deployment identities; legacy internal topic names retain their R4 schemas.

The operator must record pump requests as well as publish explicit intent.
This contract does not authenticate DDS participants or attest firmware BOOT.
"""

OPERATOR_NODE = 'fire_robot_operator'
PULSE_NODE = 'fire_pulse_controller'
PULSE_STATE_TOPIC = '/fire_mission/r4d_state'
STATIONARY_REQUEST_TOPIC = '/fire_mission/r4d_stationary_request'
FIRMWARE_PROFILE = 'N8R2_MOTOR_TRACE_TURN170_SERVO60_TICK10_PULSE6_ACK_LINK_HOME'
FIRMWARE_ELF_SHA256 = 'ce19ecd3f4fa2bbabea4c7857723a6e0ba385b6088a271136db79d74f826eee5'
SERVO_MIN_DEG = 60
SERVO_MAX_DEG = 115


def valid_gid(endpoint):
    """Humble endpoint graph GID, not callback provenance/firmware identity."""
    value = getattr(endpoint, 'endpoint_gid', None)
    if not isinstance(value, (bytes, bytearray, list, tuple)):
        return False
    return (len(value) == 24 and any(value)
            and all(type(x) is int and 0 <= x <= 255 for x in value))


def exact_endpoint(infos, node, typ, reliability):
    return int(len(infos) == 1 and infos[0].node_name == node
               and infos[0].node_namespace == '/' and infos[0].topic_type == typ
               and infos[0].qos_profile.reliability == reliability
               and valid_gid(infos[0]))
