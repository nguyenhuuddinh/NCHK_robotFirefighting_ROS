"""Desktop fire workflow only; preserve original R5 ownership and STOP audits."""
from operational_entry import LocalProcesses
from .nav_session import OUTER_INT_BUDGET
import time


class AppLocalProcesses(LocalProcesses):
    def start(self, role, argv, *, launch_leader=False):
        process = super().start(role, argv, launch_leader=launch_leader)
        if role == 'nav':
            # This stage occurs AFTER operator STOP and mission drain. Give the
            # owned wrapper time for bounded lifecycle RPC + launch shutdown.
            # Original SIGTERM/SIGKILL and all group/child audits are retained.
            process.budgets = (OUTER_INT_BUDGET, *process.budgets[1:])
        return process

    def drain(self, stop_flag=lambda: False, *, clock=time.monotonic, sleep=time.sleep, cause='FINALLY'):
        from operational_entry import atomic
        from .operator_owned_backend import OperatorOwnedBackend
        from .operator_result import read_result
        self.begin_stop(clock(), cause)
        while not self.stack.done:
            self.stack.tick(clock())
            sleep(.05)
        result = self.stack.report()
        operator = self.processes.get('operator')
        if operator is not None and isinstance(operator.io, OperatorOwnedBackend):
            try:
                row, digest = read_result(self.out / 'operator_result.json', operator, operator.io.bootstrap)
                if result['stop_confirmed'] is not True:
                    raise ValueError('APP_OPERATOR_STOP_LOG_NOT_CONFIRMED')
                result.update(runtime_result=row, runtime_result_sha256=digest,
                              runtime_failure=row['runtime_status'] == 'FAILED')
            except (OSError, ValueError, TypeError) as exc:
                result['clean'] = result['group_cleanup_clean'] = False
                result['errors'].append('APP_OPERATOR_RESULT_INVALID:' + type(exc).__name__)
        atomic(self.out / 'cleanup.json', result)
        return result
