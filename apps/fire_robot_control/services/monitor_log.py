"""Read STOP from the owned app monitor without making it an R5 ROS source."""
import re

from operational_logs import ANSI, STOP, strict_json
from operational_stream_log import Audit, StreamingOwnedLog


MONITOR_STOP = re.compile(
    r'^\[INFO\] \[\d+\.\d+\] \[fire_robot_app_stop_monitor\]: OPERATOR_STOP (\{.*\})$')


class MonitorAudit(Audit):
    def line(self, raw):
        try:
            line = ANSI.sub('', raw.decode('utf-8')).rstrip('\r\n')
        except UnicodeError:
            return super().line(raw)
        match = MONITOR_STOP.fullmatch(line)
        if match:
            self.receipts += 1
            try:
                self.proof = strict_json(match.group(1))
            except (ValueError, TypeError):
                self.error('INVALID_STOP_JSON')
            return
        if STOP.fullmatch(line):
            self.error('FOREIGN_OPERATOR_IN_MONITOR_LOG')
            return
        super().line(raw)


def start_monitor(local, argv):
    """Install the parser before any scan; retain R5 inode/exit/proof checks."""
    process = local.start('operator', argv)
    log = local.registry.logs['operator']
    if not isinstance(log, StreamingOwnedLog) or log.audit_state.offset != 0:
        raise ValueError('UNSCANNED_OWNED_MONITOR_LOG_REQUIRED')
    log.audit_state = MonitorAudit()
    return process
