"""Subprocess-only real rclpy native-delay comparison. Never launch on a robot."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from services.frontend_lease import FrontendLease
from services.operator_bootstrap import create_bootstrap_file
from services.operator_channel import OperatorChannel
from services.operator_channel_reader import OperatorChannelReader
from services.operator_process_channel import OperatorProcessChannel
from services.operator_runner import OperatorBootstrap


def run(mode, fixture):
    with tempfile.TemporaryDirectory() as tmp:
        directory=Path(tmp)
        boot=OperatorBootstrap('8518a120-a012-4db3-bbf3-f73f67aed61e','b'*32,'c'*32,'a'*64,directory/'ui.sock')
        if mode=='old_thread':
            lease=FrontendLease(boot.session,boot.generation,boot.nonce,os.getuid(),time.monotonic())
            channel=OperatorChannelReader(OperatorChannel(boot.socket_path,lease,clock=time.monotonic))
        else:
            channel=OperatorProcessChannel(boot,diagnostic_path=directory/'ui.json')
        with create_bootstrap_file(boot,directory) as private:
            child=subprocess.Popen([fixture,str(boot.socket_path),'alive',str(private.fileno())],
                pass_fds=(private.fileno(),),stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        node=None
        try:
            deadline=time.monotonic()+1.
            while not channel.lease.active and time.monotonic()<deadline:
                assert channel.poll(time.monotonic()) is None
                time.sleep(.01)
            assert channel.lease.active
            node=rclpy.create_node('app_qa_native_query')
            began=time.monotonic()
            node.get_publishers_info_by_topic('/fire_app_qa_delay')
            elapsed=time.monotonic()-began
            time.sleep(.03)
            key=channel.poll(time.monotonic())
            return dict(mode=mode,query_s=elapsed,key=key,reason=channel.lease.reason)
        finally:
            child.terminate();child.communicate(timeout=2.)
            channel.close()
            if node is not None:node.destroy_node()


if __name__=='__main__':
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and os.environ.get('ROS_DOMAIN_ID')=='211'
    rclpy.init()
    try:
        rows=[run(mode,sys.argv[1]) for mode in ('old_thread','new_process')]
        print('NATIVE_LIVENESS_RESULT='+json.dumps(rows))
        assert rows[0]['key']=='Q' and rows[0]['reason']=='FRONTEND_LEASE_EXPIRED'
        assert rows[1]['key'] is None and rows[1]['reason']=='FRONTEND_ACTIVE'
        assert all(r['query_s']>.45 for r in rows)
    finally:rclpy.shutdown()
