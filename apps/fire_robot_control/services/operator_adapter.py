"""Adapter for feeding an existing operator tick from the local channel.

No node, socket or robot is started by importing this module. A future pinned
operator entrypoint must call this on the operator's own thread and retain the
existing finally/STOP verification path. This adapter is not that entrypoint.
"""


class OperatorAdapter:
    def __init__(self, node, channel):
        self.node = node
        self.channel = channel
        self.failed = False
        self.stopping = False

    def tick(self, now):
        """Poll without blocking and deliver only X/Q to the existing node."""
        # After Q, only advance the existing STOP verifier. In particular, a
        # missing first heartbeat must not suspend verification after the lease
        # expires, and no later command may revive this session.
        if self.stopping:
            return self.node.tick(None)
        key = None
        if not self.failed:
            try:
                key = self.channel.poll(now)
                if key not in (None, "X", "Q"):
                    raise ValueError("Unexpected frontend command")
                if key == "X" and not self.channel.lease.active:
                    raise ValueError("Frontend lease not active")
            except Exception:
                # Keep a single failure latched; node's existing STOP verifier
                # must still run even if the frontend transport is broken.
                self.failed = True
                key = None
                self.node.request_shutdown("FRONTEND_CHANNEL_ERROR")
                self.stopping = True
        if not self.failed and key is None and not self.channel.lease.active:
            # ROS callbacks can still be spun by the owner, but the operator
            # must not advance its automatic policy until the actual frontend
            # event loop has authenticated with a fresh heartbeat.
            return None
        if key == "Q":
            self.stopping = True
            # Begin STOP before node.tick can enter another normal graph scan.
            self.node.request_shutdown("Q")
        return self.node.tick(key)
