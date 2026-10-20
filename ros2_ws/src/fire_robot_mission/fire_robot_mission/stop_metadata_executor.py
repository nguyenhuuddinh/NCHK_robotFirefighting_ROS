"""Humble-only opt-in adapter retaining original DDS source timestamps.

Uses the same single-threaded executor and subscription handles as stock
Humble. Only explicitly marked subscriptions receive the metadata dict;
ordinary callbacks retain stock behavior. No new polling timer or thread.
The two protected hooks must be verified against the installed Humble build.
"""

from rclpy.executors import SingleThreadedExecutor, await_or_execute


class StopMetadataExecutor(SingleThreadedExecutor):
    def _take_subscription(self, sub):
        if getattr(sub, '_fire_stop_metadata_callback', None) is None:
            return super()._take_subscription(sub)
        with sub.handle:
            return sub.handle.take_message(sub.msg_type, sub.raw)

    async def _execute_subscription(self, sub, msg):
        callback = getattr(sub, '_fire_stop_metadata_callback', None)
        if callback is None:
            return await super()._execute_subscription(sub, msg)
        if msg is not None:
            message, metadata = msg
            await await_or_execute(callback, message, metadata)
