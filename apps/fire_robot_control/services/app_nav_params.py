"""Generate a complete app Nav2 file without editing the sealed R5 config."""
import ast
import math
from pathlib import Path

import yaml


def write_params(parent, tuning, destination):
    params = yaml.safe_load(Path(parent).read_text())
    radii = yaml.safe_load(Path(tuning).read_text())
    scopes = ('local_costmap', 'global_costmap')
    if not isinstance(radii, dict) or set(radii) != set(scopes):
        raise ValueError('APP_NAV_TUNING_INVALID')
    for scope in scopes:
        settings = radii[scope]
        if not isinstance(settings, dict) or set(settings) != {'inflation_radius'}:
            raise ValueError('APP_NAV_TUNING_INVALID')
        radius = settings['inflation_radius']
        node = params[scope][scope]['ros__parameters']
        footprint = ast.literal_eval(node['footprint'])
        envelope = max(math.hypot(x, y) for x, y in footprint) + math.sqrt(2) * node['footprint_padding']
        minimum = max(envelope, params['controller_server']['ros__parameters']['FollowPath']['cost_scaling_dist'])
        if (type(radius) not in (int, float) or not math.isfinite(radius)
                or radius < minimum):
            raise ValueError('APP_NAV_INFLATION_TOO_SMALL_OR_INVALID:' + scope)
        node['inflation_layer']['inflation_radius'] = float(radius)
    Path(destination).write_text(yaml.safe_dump(params, sort_keys=False))
