import json
import os
from types import SimpleNamespace as N
from pathlib import Path
import pytest
import operational_preflight as module


@pytest.mark.parametrize('bad',[None,'route','address','existing','unknown_process','domain','xml'])
def test_preflight_reads_only_reviewed_route_and_blocks_duplicates_overrides(monkeypatch,bad):
    monkeypatch.setattr(module,'installed_sources',lambda *a:['verified'])
    for key in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_FASTRTPS_USE_QOS_FROM_XML'):monkeypatch.delenv(key,raising=False)
    if bad=='domain':monkeypatch.setenv('ROS_DOMAIN_ID','9')
    if bad=='xml':monkeypatch.setenv('RMW_FASTRTPS_USE_QOS_FROM_XML','1')
    original=Path.read_text
    monkeypatch.setattr(Path,'read_text',lambda p,*a,**k:'down' if str(p).startswith('/sys/class/net/') else original(p,*a,**k))
    calls=[]
    def run(argv,**kw):
        calls.append(argv)
        if argv[0]=='pgrep':return N(returncode=0 if bad=='existing' else 2 if bad=='unknown_process' else 1,stdout='')
        if 'address' in argv:return N(returncode=0,stdout=json.dumps([dict(addr_info=[dict(local='192.168.1.2' if bad=='address' else '10.0.0.253')])]))
        return N(returncode=0,stdout=json.dumps([dict(dev='eth0' if bad=='route' else 'wlp8s0')]))
    if bad is None:
        assert module.preflight('/fake','laptop',run=run)['live'] is False
        assert len(calls)==3
    else:
        with pytest.raises(ValueError):module.preflight('/fake','laptop',run=run)
    assert all(a[0] in ('pgrep','ip') for a in calls)


def test_installed_config_mismatch_is_blocked_before_robot_start(tmp_path,monkeypatch):
    monkeypatch.setitem(module.PACKAGES,'laptop',('fake',))
    src=tmp_path/'ros2_ws/src/fake/config';src.mkdir(parents=True);(src/'params.yaml').write_text('expected')
    share=tmp_path/'installed';(share/'config').mkdir(parents=True);(share/'config/params.yaml').write_text('old')
    monkeypatch.setattr(module,'get_package_share_directory',lambda name:str(share))
    with pytest.raises(ValueError,match='INSTALLED_CONFIG_NOT_CANDIDATE'):module.installed_sources(tmp_path,'laptop')
