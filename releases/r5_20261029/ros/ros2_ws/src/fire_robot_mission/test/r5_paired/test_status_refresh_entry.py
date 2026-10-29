"""R5 explicitly selects the local Nav2 status-refresh implementation."""
from pathlib import Path
import pytest
import r5_laptop_session as laptop


@pytest.mark.parametrize('gate2',[False,True])
def test_R5_nav_command_requests_real_status_refresh_without_changing_control_route(gate2):
    c=laptop.commands(Path('/tmp/map.yaml'),Path('/tmp/nav.rviz'),gate2=gate2)
    assert c['nav'].count('refresh_bt_action_status:=true')==1
    assert 'nav2_composed_shadow.launch.py' in c['nav']
    assert not any('goal' in x or 'cancel' in x for x in c['nav'])


@pytest.mark.parametrize('refreshed',[False,True])
def test_static_inventory_rejects_old_container_even_when_ELF_architecture_matches(monkeypatch,tmp_path,refreshed):
    import r5_install_check as inventory
    binary=tmp_path/'container'
    binary.write_bytes(b'ELF' + (inventory.NAV_STATUS_REFRESH_MARKER if refreshed else b'old_baseline'))
    monkeypatch.setattr(inventory,'checked_elf',lambda *a,**k:dict(machine='x86_64'))
    if refreshed:assert inventory.checked_nav_container(binary,'x86_64')['actual_status_refresh_available']
    else:
        with pytest.raises(ValueError,match='NAV2_STATUS_REFRESH_BUILD_REQUIRED'):
            inventory.checked_nav_container(binary,'x86_64')
