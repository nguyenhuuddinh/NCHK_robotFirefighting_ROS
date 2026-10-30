"""QA-only launch prefix: allow a debugger for this one localhost inferior."""
import ctypes
import os
from pathlib import Path
import sys

assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID') == '216'
assert Path(sys.argv[1]).name == 'fire_robot_component_container_isolated'
libc = ctypes.CDLL(None, use_errno=True)
if libc.prctl(ctypes.c_int(0x59616d61), ctypes.c_ulong(-1), 0, 0, 0) != 0:
    raise OSError(ctypes.get_errno(), 'QA debugger permission failed')
os.execv(sys.argv[1], sys.argv[1:])
