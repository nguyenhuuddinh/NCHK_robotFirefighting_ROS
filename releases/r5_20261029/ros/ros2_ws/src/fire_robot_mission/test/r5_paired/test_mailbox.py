import json
import os
from pathlib import Path
import uuid
import pytest
from operational_peer import Mailbox,call
from test_pi_protocol import B


def mailbox(tmp_path):
    root=tmp_path/'pi';root.mkdir(mode=0o700);(root/'requests').mkdir();(root/'responses').mkdir()
    binding={**B,'root':str(root)};(root/'pairing.json').write_text(json.dumps(binding))
    return root,binding,Mailbox(root,binding)


def test_exact_new_request_response_not_stale_saved_state(monkeypatch,tmp_path):
    root,b,m=mailbox(tmp_path);now=[100.];monkeypatch.setattr('operational_peer.time.monotonic',lambda:now[0])
    def serve(s):
        for p,r in m.requests():m.reply(p,r,dict(session=b['session'],phase='WAIT_LAPTOP'))
        now[0]+=s
    result=call(root,b,'STATE',clock=lambda:now[0],sleep=serve)
    assert result['state']['phase']=='WAIT_LAPTOP' and not list((root/'requests').glob('*.json'))


def test_timeout_withdraws_start_and_server_rejects_expired_mutation(monkeypatch,tmp_path):
    root,b,m=mailbox(tmp_path);now=[100.];monkeypatch.setattr('operational_peer.time.monotonic',lambda:now[0])
    with pytest.raises(TimeoutError):call(root,b,'START',timeout=.1,clock=lambda:now[0],sleep=lambda s:now.__setitem__(0,now[0]+s))
    assert not list((root/'requests').glob('*.json'))
    p=root/'requests'/(str(uuid.uuid4())+'.json');p.write_text(json.dumps(dict(binding=b,command='START',request=p.stem,expires_mono=99)))
    assert list(m.requests())==[] and p.with_suffix('.rejected').exists()


@pytest.mark.parametrize('bad',['secret','packet','boot_id','runner_start_ticks','command','request'])
def test_foreign_or_invalid_request_rejected_not_executed(tmp_path,bad):
    root,b,m=mailbox(tmp_path);value=dict(binding=dict(b),command='START',request=str(uuid.uuid4()),expires_mono=10**12)
    if bad in value:value[bad]='ARM' if bad=='command' else 'invalid'
    else:value['binding'][bad]='other'
    p=root/'requests'/(str(uuid.uuid4())+'.json');p.write_text(json.dumps(value))
    assert list(m.requests())==[] and p.with_suffix('.rejected').exists()


def test_completed_receipt_only_for_same_binding_and_never_start_gate(tmp_path):
    root,b,m=mailbox(tmp_path);(root/'DONE.json').write_text(json.dumps(dict(session=b['session'],phase='DONE')))
    assert call(root,b,'STATE')['state']['phase']=='DONE'
    wrong={**b,'runner_start_ticks':55}
    with pytest.raises(ValueError):call(root,wrong,'STATE')


def test_session_directory_must_be_private_and_not_symlink(tmp_path):
    root,b,m=mailbox(tmp_path);root.chmod(0o755)
    with pytest.raises(ValueError):Mailbox(root,b)
    root.chmod(0o700);link=tmp_path/'link';link.symlink_to(root)
    with pytest.raises(ValueError):Mailbox(link,b)
