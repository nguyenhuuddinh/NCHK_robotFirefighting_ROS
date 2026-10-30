"""App-only operator entrypoint with independent UI liveness and R5 STOP proof.

The generated app node preserves R5 interfaces and adds current-time checks.
The standalone sealed R5 entrypoint is never modified by this module.
"""
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import re
import signal
import sys
import time
import uuid

from .frontend_lease import FrontendLease
from .operator_channel import OperatorChannel
from .operator_loop import operator_loop, drain_operator_stop


@dataclass(frozen=True)
class OperatorBootstrap:
    session: str
    generation: str
    nonce: str = field(repr=False)
    packet: str
    socket_path: Path

    def validate(self):
        try:
            session_uuid = uuid.UUID(self.session) if isinstance(self.session, str) else None
            valid_session = session_uuid is not None and session_uuid.int != 0 and str(session_uuid) == self.session
        except ValueError:
            valid_session = False
        if (not valid_session or not isinstance(self.generation, str)
                or re.fullmatch(r"[a-f0-9]{32}", self.generation) is None
                or not isinstance(self.nonce, str) or re.fullmatch(r"[a-f0-9]{32}", self.nonce) is None
                or not isinstance(self.packet, str) or re.fullmatch(r"[a-f0-9]{64}", self.packet) is None
                or not isinstance(self.socket_path, Path) or not self.socket_path.is_absolute()
                or "\0" in str(self.socket_path)):
            raise ValueError("Invalid operator bootstrap")


def run_integrated(bootstrap, args=None, *, ros=None, node_factory=None,
                   channel_factory=OperatorChannel, clock=time.monotonic, diagnostic_path=None, result_path=None):
    """Run the candidate loop; do not call until the packet gate is complete.

    `ros` and `node_factory` are injectable for offline tests. Identity must
    come from a private, versioned session bootstrap controlled by the runner.
    Existing ROS context, node and signal ownership must not be shared.
    """
    if not isinstance(bootstrap, OperatorBootstrap):
        raise TypeError("OperatorBootstrap required")
    bootstrap.validate()
    if ros is None:
        import rclpy as ros
        from rclpy.signals import SignalHandlerOptions
        signal_handler_options = SignalHandlerOptions.NO
    else:
        # Injected runtimes are used only by offline tests.
        signal_handler_options = None
    if node_factory is None:
        from .app_operator_source import load_operator_node
        node_factory = load_operator_node()
    if ros.ok():
        raise RuntimeError("ROS context already active")

    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    stopping = [False]
    node = None
    channel = None
    rc = 0
    def stop(*_):
        stopping[0] = True
    for sig in previous:
        signal.signal(sig, stop)
    try:
        ros.init(args=args, signal_handler_options=signal_handler_options)
        node = node_factory()
        # Never allow a second OpenCV operator or an unbound automatic session.
        if (node.view is not None or node.get_parameter("automatic_mode").value is not True
                or node.get_parameter("automatic_runner_session").value != bootstrap.session
                or node.get_parameter("automatic_packet").value != bootstrap.packet):
            raise ValueError("Operator profile or session differs from bootstrap")
        if channel_factory is OperatorChannel:
            from .operator_process_channel import OperatorProcessChannel
            channel = OperatorProcessChannel(bootstrap, clock=clock, diagnostic_path=diagnostic_path)
        else:
            lease = FrontendLease(bootstrap.session, bootstrap.generation, bootstrap.nonce,
                                  os.getuid(), clock())
            channel = channel_factory(bootstrap.socket_path, lease)
        report = operator_loop(node, channel, ros=ros, clock=clock, stopping=lambda: stopping[0])
        if report is None:
            rc = 2
    except BaseException as exc:
        rc = 2
        if node is not None:
            node.get_logger().error("Integrated operator exception: " + type(exc).__name__)
        else:
            print("Integrated operator failed before startup: " + type(exc).__name__, file=sys.stderr)
    finally:
        try:
            if node is not None:
                try:
                    node.request_shutdown("FINALLY")
                    drain_operator_stop(node, ros=ros, clock=clock)
                    if not node.stop_verification.done:
                        node.stop_verification.abort("ROS_CONTEXT_LOST")
                    if not node.stop_verification.confirmed:
                        rc = 2
                    node.get_logger().info("OPERATOR_STOP " + json.dumps(node.stop_verification.report(clock())))
                except BaseException as exc:
                    rc = 2
                    node.get_logger().error("Operator stop verification failed: " + type(exc).__name__)
                finally:
                    try:
                        if node.view is not None:
                            node.view.close()
                    except BaseException as exc:
                        rc = 2
                        node.get_logger().error("Operator view cleanup failed: " + type(exc).__name__)
                    try:
                        node.destroy_node()
                    except BaseException as exc:
                        rc = 2
                        node.get_logger().error("Operator node cleanup failed: " + type(exc).__name__)
        finally:
            try:
                if channel is not None:
                    try:
                        channel.close()
                    except BaseException:
                        rc = 2
                if ros.ok():
                    try:
                        ros.shutdown()
                    except BaseException:
                        rc = 2
            finally:
                for sig, handler in previous.items():
                    signal.signal(sig, handler)
    fault = None if node is None else node.core.fault
    # Process exit represents STOP/teardown completion. A failed mission remains
    # a failed SESSION through the mandatory bound result consumed by the runner.
    # Without a persistent record, never silently turn a fault into success.
    if fault is not None and result_path is None:
        rc = 2
    if result_path is not None:
        from .operator_result import write_result
        try:
            write_result(result_path, bootstrap, fault=fault, rc=rc,
                         stop_confirmed=node is not None and node.stop_verification.confirmed is True)
        except (OSError, ValueError, TypeError):
            rc = 2
            print('APP_OPERATOR_RESULT_WRITE_FAILED', file=sys.stderr, flush=True)
    return rc
