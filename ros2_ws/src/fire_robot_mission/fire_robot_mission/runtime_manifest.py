"""Explicit shadow rosters; never infer an omitted owner from discovery loss."""

from dataclasses import dataclass


OWNERS = ('live_nav_stop_gateway', 'live_alignment_takeover',
          'ir_approach_live', 'pump_demo_controller')


@dataclass(frozen=True)
class RuntimeManifest:
    name: str
    required: tuple

    @property
    def omitted(self):
        return tuple(owner for owner in OWNERS if owner not in self.required)


_MANIFESTS = {
    'all_four_external_v1': RuntimeManifest('all_four_external_v1', OWNERS),
    'wireless_static_observation_v1': RuntimeManifest('wireless_static_observation_v1', OWNERS[:3]),
    'io_nav_static_no_owners_v1': RuntimeManifest('io_nav_static_no_owners_v1', ()),
}


def runtime_manifest(name):
    if type(name) is not str or name not in _MANIFESTS:
        raise ValueError('unknown explicit shadow runtime manifest')
    return _MANIFESTS[name]
