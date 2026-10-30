"""App cleanup alone may read a bound timeout drain; startup stays strict."""
import json
from types import SimpleNamespace as Obj
from unittest.mock import Mock
import unittest

from services.app_peer import AppPeer
from operational_peer import PeerTransportError


class AppDrainTransportTests(unittest.TestCase):
    def peer(self, **state_changes):
        binding=dict(session='session',secret='fixture',packet='a'*64,boot_id='boot',
                     runner_pid=12,runner_start_ticks=34,root='/fake')
        state=dict(session='session',phase='WAIT_LAPTOP_DRAIN',error='PI_SEAL_HEALTH_TIMEOUT',
                   sealed=False,app_prearm_drain=True)
        state.update(state_changes)
        reply=dict(binding=dict(binding),state=state)
        run=Mock(side_effect=lambda *args,**kw:Obj(returncode=0,stdout=json.dumps(reply),stderr=''))
        peer=AppPeer(binding,'/unused',run=run)
        return peer,reply,run

    def test_startup_still_rejects_drain_error(self):
        peer,_,_=self.peer()
        with self.assertRaisesRegex(RuntimeError,'PI_RUNNER_ERROR'):
            peer.startup('STATE',sleep=lambda _:None)

    def test_bound_wait_and_stopping_keep_original_error_during_cleanup(self):
        for phase in ('WAIT_LAPTOP_DRAIN','STOPPING'):
            with self.subTest(phase=phase):
                peer,_,run=self.peer(phase=phase);peer._stopping=True
                self.assertEqual(peer.request('STATE')['error'],'PI_SEAL_HEALTH_TIMEOUT')
                self.assertTrue(run.call_args.kwargs['timeout']==9)

    def test_drain_wrong_identity_or_error_cannot_pass(self):
        for key,value in [('sealed',True),('app_prearm_drain',False),('phase','IO_STARTING'),
                          ('error','UNKNOWN_FAILURE'),('request_error','REJECTED')]:
            with self.subTest(key=key):
                peer,_,_=self.peer(**{key:value});peer._stopping=True
                with self.assertRaises(RuntimeError):peer.request('STATE')
        for key in ('session','secret','packet','boot_id','runner_pid','runner_start_ticks','root'):
            with self.subTest(binding=key):
                peer,reply,_=self.peer();peer._stopping=True;reply['binding'][key]='foreign'
                with self.assertRaises(ValueError):peer.request('STATE')
        peer,reply,_=self.peer();peer._stopping=True;reply['state']['session']='foreign'
        with self.assertRaises(ValueError):peer.request('STATE')

    def test_cleanup_cannot_send_mutation(self):
        for command in ('START','SEAL','RETRY_PREARM'):
            peer,_,run=self.peer();peer._stopping=True
            with self.assertRaisesRegex(ValueError,'APP_DRAIN_STOP_ONLY'):peer.request(command)
            run.assert_not_called()

    def test_transport_failure_resets_cleanup_scope_and_does_not_expose_output(self):
        peer,_,run=self.peer();run.side_effect=OSError('private-transport-text')
        peer._stopping=True
        with self.assertRaises(PeerTransportError) as caught:peer.request('STATE')
        self.assertNotIn('private-transport-text',str(caught.exception))
        peer._stopping=False
        peer.request=Mock(side_effect=ValueError('invalid'))
        now=[0.]
        result=peer.stop(clock=lambda:now[0],sleep=lambda dt:now.__setitem__(0,now[0]+dt))
        self.assertFalse(peer._stopping)
        self.assertFalse(result['confirmed']);self.assertFalse(result['cleanup_confirmed'])
