"""ROS-independent raw serial session state machine, shared by worker and tests."""
from collections import deque
import json
import threading
import time
from fire_robot_bringup.serial_protocol import SerialProtocolV2
from fire_robot_bringup.serial_fault_trace import classify_esp_uptime
from fire_robot_bringup.timing_diagnostics import TimingDiagnostics


class SerialTransportCore:
    def init_transport(self, port, baudrate, first_state_timeout_s,
                       pump_command_timeout_s, serial_cls,
                       diagnostics_enabled=True, fault_trace_events=0):
        if (isinstance(fault_trace_events, bool)
                or not isinstance(fault_trace_events, int)
                or not 0 <= fault_trace_events <= 16):
            raise ValueError('fault_trace_events must be an integer 0-16')
        self.port = port
        self.baudrate = baudrate
        self.first_state_timeout_s = first_state_timeout_s
        self.pump_command_timeout_s = pump_command_timeout_s
        self.serial_cls = serial_cls
        self.protocol = SerialProtocolV2()
        self.ser = None

        self.state_lock = threading.Lock()
        self.tx_lock = threading.Lock()
        self.publish_lock = threading.Lock()
        self.node_destroyed = False
        self.stop_request = False
        self._session_state = 'BACKOFF'
        self.open_generation = -1
        self.close_ownership = {
            'task': None,
            'handle': None,
            'result': None,
            'retry_deadline': 0.0,
            'backoff': 0.05,
            'generation': -1,
            'cancel_called': False,
            'deadline': None,
            'pending': [],
        }

        self.session_ready = False
        self.session_epoch = 0
        self.telemetry_healthy = False
        self.state_stale_timeout_ms = 500
        self.latest_usb_diagnostic = None
        self._boot_logged_epoch = None
        # A bounded, event-only breadcrumb survives the 180 s USB frame
        # capture window and works with periodic diagnostics disabled.
        self._usb_recoveries_seen = None
        self._usb_recovery_events_logged = 0
        # Opt-in, bounded event capture. No extra timer, thread or frame queue.
        self.fault_trace_events = fault_trace_events
        self.fault_trace_count = 0
        self._fault_trace_last_state = None
        self._fault_trace_pending = None
        # Recent read calls stay in memory only while fault tracing is armed.
        # A fault logs one bounded summary, not a stream of normal read calls.
        self._fault_read_window = (
            deque(maxlen=128) if fault_trace_events else None)
        self._fault_last_nonempty_read_ns = None

        self.pending_cmd = None
        self.pending_pump = None
        self.pending_fire = None

        self.pump_state = 0
        self.cmd_version = 0
        self.pump_version = 0
        self.fire_version = 0
        self.pump_written_version = 0
        self.last_pump_time = 0.0
        self.last_pump_command_time = None
        self.session_started_mono = 0.0
        self.last_state_time_mono = 0.0
        self.last_env_time_mono = 0.0
        self.last_ir_time_mono = 0.0

        self.reconnect_backoff = 0.0
        self.next_reconnect_mono = 0.0
        self.latest_state = None
        self.latest_env = None
        self.latest_ir = None
        self.latest_actuator = None

        self.tx_fail_count = 0
        self.tx_partial_count = 0
        self.close_fail_count = 0
        self.reconnect_count = 0
        self.reconnect_total = 0
        self.first_state_timeout_count = 0
        self.read_fail_count = 0
        self.bootstrap_fail_count = 0
        self.open_fail_count = 0
        self.open_attempt_count = 0
        self.open_success_count = 0
        self.cmd_drop_count = 0
        self.telemetry_drop_count = 0
        self.telemetry_publish_count = 0
        self.telemetry_publish_fail_count = 0
        self.last_failure_reason = 'none'
        self.last_quarantine_mono = None

        # Windowed timing diagnostics only; these never change admission or
        # the 500 ms telemetry fail-closed deadline.
        self.read_max_ms = 0.0
        self.tx_max_ms = 0.0
        self.worker_gap_max_ms = 0.0
        self.state_gap_max_ms = 0.0
        self.last_worker_tick_mono = None

        self.state_publish_max_age = 0.0
        self.env_publish_max_age = 0.0
        self.diagnostics_enabled = bool(diagnostics_enabled)
        self.odom_timing = TimingDiagnostics(enabled=self.diagnostics_enabled)

        self.running = True

    def _emit_fault_trace(self, event):
        self.get_logger().info('serial_fault_trace ' + json.dumps(
            event, sort_keys=True, separators=(',', ':'), ensure_ascii=True))

    def _transition_state(self, new_state):
        """Transition the transport state while ``state_lock`` is held."""
        valid = {
            'BACKOFF': ['OPENING', 'SHUTTING_DOWN'],
            'OPENING': [
                'WAIT_FIRST_STATE', 'BACKOFF', 'CLOSING', 'SHUTTING_DOWN'],
            'WAIT_FIRST_STATE': ['HEALTHY', 'CLOSING', 'SHUTTING_DOWN'],
            'HEALTHY': ['CLOSING', 'SHUTTING_DOWN'],
            'CLOSING': ['BACKOFF', 'SHUTTING_DOWN'],
            'SHUTTING_DOWN': []
        }
        if new_state in valid.get(getattr(self, '_session_state', ''), []) or \
                (getattr(self, '_session_state', '') == 'CLOSING' and new_state == 'CLOSING'):
            self._session_state = new_state


    def _increment_counter(self, name, amount=1):
        """Increment a diagnostic counter atomically."""
        with self.state_lock:
            setattr(self, name, getattr(self, name, 0) + amount)


    def _record_io_duration(self, name, started_mono):
        """Record a serial operation duration without changing its result."""
        if not getattr(self, 'diagnostics_enabled', True):
            return
        elapsed_ms = max(0.0, (time.monotonic() - started_mono) * 1000.0)
        with self.state_lock:
            setattr(self, name, max(getattr(self, name, 0.0), elapsed_ms))


    def _timed_read(self, handle, size):
        """Measure read latency, including an exceptional return."""
        if not getattr(self, 'diagnostics_enabled', True):
            return handle.read(size)
        started_mono = time.monotonic()
        try:
            return handle.read(size)
        finally:
            self._record_io_duration('read_max_ms', started_mono)


    def _read_with_fault_trace(self, handle, size, epoch):
        """Keep the second before a fault, including empty reads and errors."""
        if self._fault_read_window is None:
            return self._timed_read(handle, size)
        started_ns = time.monotonic_ns()
        try:
            data = self._timed_read(handle, size)
        except Exception:
            self._record_fault_read(started_ns, time.monotonic_ns(),
                                    0, True, epoch)
            raise
        self._record_fault_read(started_ns, time.monotonic_ns(),
                                len(data), False, epoch)
        return data


    def _record_fault_read(self, started_ns, ended_ns, byte_count,
                           error, epoch):
        with self.state_lock:
            if self._fault_read_window is None or self.session_epoch != epoch:
                return
            self._fault_read_window.append(
                (started_ns, ended_ns, byte_count, error))
            if byte_count:
                self._fault_last_nonempty_read_ns = ended_ns


    def _fault_read_summary_locked(self, fault_ns):
        """Summarize opt-in read activity; caller holds state_lock."""
        window = self._fault_read_window
        if window is None:
            return None
        records = list(window)
        cutoff_ns = fault_ns - 1_000_000_000
        calls = [sample for sample in records if sample[0] >= cutoff_ns]
        gaps_ms = [
            (current[0] - previous[0]) / 1e6
            for previous, current in zip(records, records[1:])
            if current[0] >= cutoff_ns]
        last_data_ns = self._fault_last_nonempty_read_ns
        return {
            'window_ms': 1000,
            'capacity': window.maxlen,
            'calls': len(calls),
            'empty': sum(not count and not error
                         for _, _, count, error in calls),
            'errors': sum(error for _, _, _, error in calls),
            'bytes': sum(count for _, _, count, _ in calls),
            'span_ms': round((calls[-1][1] - calls[0][0]) / 1e6, 3)
                       if calls else None,
            'max_read_ms': round(max((end - start) / 1e6
                                     for start, end, _, _ in calls), 3)
                           if calls else None,
            'max_start_gap_ms': round(max(gaps_ms), 3) if gaps_ms else None,
            'last_read_end_age_ms': round(
                max(0, fault_ns - calls[-1][1]) / 1e6, 3)
                if calls else None,
            'last_nonempty_end_age_ms': round(
                max(0, fault_ns - last_data_ns) / 1e6, 3)
                if last_data_ns is not None else None,
        }


    def _timed_write(self, handle, frame):
        """Measure write latency without logging the command payload."""
        if not getattr(self, 'diagnostics_enabled', True):
            return handle.write(frame)
        started_mono = time.monotonic()
        try:
            return handle.write(frame)
        finally:
            self._record_io_duration('tx_max_ms', started_mono)


    def cmd_vel_callback(self, msg):
        """Handle /cmd_vel."""
        with self.state_lock:
            if not getattr(self, 'session_ready', False) or \
                    (not getattr(self, 'telemetry_healthy', False) and
                     (msg.linear.x != 0.0 or msg.angular.z != 0.0)):
                self.cmd_drop_count += 1
                return
            self.cmd_version += 1
            self.pending_cmd = ((msg.linear.x, msg.angular.z),
                                self.session_epoch, self.cmd_version)


    def fire_target_callback(self, msg):
        """Handle /fire_target."""
        with self.state_lock:
            if not getattr(self, 'session_ready', False) or \
                    not getattr(self, 'telemetry_healthy', False):
                self.cmd_drop_count += 1
                return
            self.fire_version += 1
            self.pending_fire = ((msg.x, msg.y), self.session_epoch, self.fire_version)


    def pump_cmd_callback(self, msg):
        """Handle /pump_cmd."""
        pump_req = 1 if msg.data else 0
        with self.state_lock:
            if not getattr(self, 'session_ready', False) or \
                    (not getattr(self, 'telemetry_healthy', False) and pump_req != 0):
                self.cmd_drop_count += 1
                return
            self.pump_version += 1
            self.pending_pump = (pump_req, self.session_epoch, self.pump_version)
            self.last_pump_command_time = time.monotonic()


    def close_serial(self):
        """Initiate or reap a serial close task."""
        with self.state_lock:
            self.session_ready = False
            self.telemetry_healthy = False
            self.pending_cmd = None
            self.pending_pump = None
            self.pending_fire = None
            self.latest_actuator = None

            if getattr(self, '_session_state', '') not in ('CLOSING', 'SHUTTING_DOWN'):
                self._transition_state('CLOSING')
                if self.reconnect_backoff <= 0.0:
                    self.reconnect_backoff = 0.1
                else:
                    self.reconnect_backoff = min(1.0, self.reconnect_backoff * 1.5)
                self.next_reconnect_mono = time.monotonic() + self.reconnect_backoff

            handle = self.ser
            self.ser = None

            if handle is not None:
                self._enqueue_close_handle_locked(
                    handle, self.session_epoch, 'close:Requested')

            self._promote_pending_close_locked()

            if self.close_ownership.get('handle') is None:
                return 'CLOSED'

            if self.close_ownership.get('result') is not None:
                res = self.close_ownership['result']
                if isinstance(res, Exception):
                    now = time.monotonic()
                    if (not getattr(self, '_shutdown_started', False)
                            and now < self.close_ownership.get('retry_deadline', 0.0)):
                        return 'IN_PROGRESS'
                    self.close_ownership['result'] = None
                    self.close_ownership['task'] = None
                    backoff = self.close_ownership.get(
                        'backoff', 0.05) * 1.5
                    self.close_ownership['backoff'] = min(
                        2.0, backoff)
                else:
                    self._clear_close_owner_locked()
                    self._promote_pending_close_locked()
                    if self.close_ownership.get('handle') is not None:
                        return 'IN_PROGRESS'
                    return 'CLOSED'

            if self.close_ownership.get('task') is None:
                shutdown_deadline = self.close_ownership.get('deadline')
                if (shutdown_deadline is not None
                        and time.monotonic() >= shutdown_deadline):
                    return 'IN_PROGRESS'

                current_handle = self.close_ownership['handle']
                current_gen = self.close_ownership['generation']

                def close_task():
                    result = None
                    trace_id = None
                    close_start_ns = None
                    cancel_ms = None
                    tx_lock_wait_ms = None
                    flush_ms = None
                    close_call_ms = None
                    with self.state_lock:
                        trace = self._fault_trace_pending
                        if (trace is not None
                                and trace['fault_epoch'] == current_gen
                                and trace.get('close_attempts', 0) < 4):
                            trace['close_attempts'] = trace.get(
                                'close_attempts', 0) + 1
                            trace_id = trace['id']
                            close_start_ns = time.monotonic_ns()
                        cancel_called = self.close_ownership.get('cancel_called', False)
                        if not cancel_called and self.close_ownership.get(
                                'generation') == current_gen:
                            self.close_ownership['cancel_called'] = True
                        else:
                            cancel_called = True

                    if trace_id is not None:
                        self._emit_fault_trace({
                            'phase': 'close_begin', 'id': trace_id,
                            'epoch': current_gen,
                            'mono_ns': close_start_ns,
                            'wall_ns': time.time_ns(),
                        })

                    deadline_expired = (
                        shutdown_deadline is not None
                        and time.monotonic() >= shutdown_deadline)
                    if deadline_expired:
                        result = TimeoutError('Shutdown deadline exhausted')

                    if not cancel_called and result is None:
                        cancel_start_ns = time.monotonic_ns() if trace_id else None
                        if getattr(current_handle, 'cancel_read', None):
                            try:
                                current_handle.cancel_read()
                            except Exception as e:
                                with self.state_lock:
                                    self.last_failure_reason = (
                                        f"cancel_read:{type(e).__name__}:{e}")
                        if getattr(current_handle, 'cancel_write', None):
                            try:
                                current_handle.cancel_write()
                            except Exception as e:
                                with self.state_lock:
                                    r = getattr(self, 'last_failure_reason', '')
                                    self.last_failure_reason = (
                                        f"{r} cancel_write:{type(e).__name__}:{e}")
                        if trace_id is not None:
                            cancel_ms = round((time.monotonic_ns()
                                               - cancel_start_ns) / 1e6, 3)

                    tx_acq = False
                    try:
                        if result is None and hasattr(self, 'tx_lock'):
                            lock_start_ns = time.monotonic_ns() if trace_id else None
                            timeout = 0.5
                            if shutdown_deadline is not None:
                                timeout = min(
                                    timeout,
                                    max(0.0, shutdown_deadline - time.monotonic()))
                            tx_acq = self.tx_lock.acquire(timeout=timeout)
                            if trace_id is not None:
                                tx_lock_wait_ms = round((time.monotonic_ns()
                                                         - lock_start_ns) / 1e6, 3)
                            if not tx_acq:
                                raise TimeoutError(
                                    'Could not acquire tx_lock before deadline')
                        if (result is None and shutdown_deadline is not None
                                and time.monotonic() >= shutdown_deadline):
                            raise TimeoutError('Shutdown deadline exhausted')
                        if result is None:
                            # Linux cdc_acm drains queued TX during close().
                            # Discard it first so a stalled USB endpoint cannot
                            # hold the close worker in tty_wait_until_sent.
                            reset_output = getattr(
                                current_handle, 'reset_output_buffer', None)
                            if reset_output is not None:
                                flush_start_ns = time.monotonic_ns() if trace_id else None
                                try:
                                    reset_output()
                                except Exception as e:
                                    with self.state_lock:
                                        self.last_failure_reason = (
                                            'close_flush:'
                                            f'{type(e).__name__}:{e}')
                                if trace_id is not None:
                                    flush_ms = round((time.monotonic_ns()
                                                      - flush_start_ns) / 1e6, 3)
                            close_call_start_ns = time.monotonic_ns() if trace_id else None
                            try:
                                current_handle.close()
                            finally:
                                if trace_id is not None:
                                    close_call_ms = round((time.monotonic_ns()
                                                           - close_call_start_ns) / 1e6, 3)
                            result = True
                    except TimeoutError as e:
                        result = e
                    except Exception as e:
                        result = e
                    finally:
                        if tx_acq:
                            self.tx_lock.release()

                    with self.state_lock:
                        if (self.close_ownership.get('handle') is current_handle
                                and self.close_ownership.get(
                                    'generation') == current_gen):
                            self.close_ownership['result'] = result
                            if isinstance(result, Exception):
                                if not isinstance(result, TimeoutError):
                                    self.close_fail_count += 1
                                self.last_failure_reason = (
                                    f"close:{type(result).__name__}:{result}")
                                self.close_ownership['retry_deadline'] = (
                                    time.monotonic()
                                    + self.close_ownership.get(
                                        'backoff', 0.05))
                        else:
                            self.get_logger().error(
                                'Close ownership changed while close task ran')

                    if trace_id is not None:
                        self._emit_fault_trace({
                            'phase': 'close_end', 'id': trace_id,
                            'epoch': current_gen,
                            'wall_ns': time.time_ns(),
                            'duration_ms': round((time.monotonic_ns()
                                                  - close_start_ns) / 1e6, 3),
                            'cancel_ms': cancel_ms,
                            'tx_lock_wait_ms': tx_lock_wait_ms,
                            'flush_ms': flush_ms,
                            'close_call_ms': close_call_ms,
                            'result': ('ok' if result is True else
                                       type(result).__name__),
                        })

                task_to_start = threading.Thread(
                    target=close_task, daemon=True)
                self.close_ownership['task'] = task_to_start
                task_to_start.start()

        return 'IN_PROGRESS'


    def _clear_close_owner_locked(self):
        """Clear the current close owner while ``state_lock`` is held."""
        self.close_ownership['task'] = None
        self.close_ownership['handle'] = None
        self.close_ownership['result'] = None
        self.close_ownership['retry_deadline'] = 0.0
        self.close_ownership['backoff'] = 0.05
        self.close_ownership['generation'] = -1
        self.close_ownership['cancel_called'] = False
        self.close_ownership['deadline'] = None


    def _install_close_owner_locked(self, handle, generation):
        """Install one handle as close owner while ``state_lock`` is held."""
        self.close_ownership['handle'] = handle
        self.close_ownership['generation'] = generation
        self.close_ownership['retry_deadline'] = 0.0
        self.close_ownership['backoff'] = 0.05
        self.close_ownership['result'] = None
        self.close_ownership['task'] = None
        self.close_ownership['cancel_called'] = False
        self.close_ownership['deadline'] = (
            getattr(self, '_shutdown_deadline_mono', None)
            if getattr(self, '_shutdown_started', False) else None)


    def _enqueue_close_handle_locked(self, handle, generation, reason):
        """Queue a unique handle without replacing existing ownership."""
        if reason:
            self.last_failure_reason = reason
        if handle is None or getattr(handle, 'closed', False):
            return

        owner = self.close_ownership.get('handle')
        if owner is handle:
            return
        pending = self.close_ownership['pending']
        if any(entry[0] is handle for entry in pending):
            return
        if owner is None:
            self._install_close_owner_locked(handle, generation)
        else:
            pending.append((handle, generation))


    def _promote_pending_close_locked(self):
        """Promote the next still-open quarantined handle."""
        if self.close_ownership.get('handle') is not None:
            return
        pending = self.close_ownership['pending']
        while pending:
            handle, generation = pending.pop(0)
            if not getattr(handle, 'closed', False):
                self._install_close_owner_locked(handle, generation)
                return


    def _quarantine_handle(self, handle, reason=""):
        """Quarantine a serial handle into close_ownership."""
        if handle is None:
            return

        trace_event = None
        with self.state_lock:
            now_mono = time.monotonic()
            old_state = self._session_state
            epoch = self.session_epoch
            last_state = self.last_state_time_mono
            state_age_ms = (
                max(0.0, (now_mono - last_state) * 1000.0)
                if last_state > 0.0 else -1.0)
            read_max_ms = self.read_max_ms
            tx_max_ms = self.tx_max_ms
            worker_gap_max_ms = self.worker_gap_max_ms
            if reason:
                self.last_failure_reason = reason
            self.last_quarantine_mono = now_mono
            if (self.fault_trace_events and
                    self._fault_trace_pending is None and
                    self.fault_trace_count < self.fault_trace_events):
                self.fault_trace_count += 1
                trace = {
                    'id': self.fault_trace_count,
                    'fault_epoch': epoch,
                    'fault_mono_ns': int(now_mono * 1_000_000_000),
                    'fault_wall_ns': time.time_ns(),
                    'last_state': self._fault_trace_last_state,
                    'reason': str(reason)[:200],
                    'reader_before_fault': self._fault_read_summary_locked(
                        int(now_mono * 1_000_000_000)),
                }
                self._fault_trace_pending = trace
                trace_event = dict(phase='fault', **trace)

            self.session_ready = False
            self.telemetry_healthy = False
            self.pending_cmd = None
            self.pending_pump = None
            self.pending_fire = None
            self.latest_actuator = None

            if self.ser is handle:
                self.ser = None

            self._enqueue_close_handle_locked(
                handle, self.session_epoch, reason)

            if getattr(self, '_session_state', '') not in ('CLOSING', 'SHUTTING_DOWN'):
                self._transition_state('CLOSING')

            if self.reconnect_backoff <= 0.0:
                self.reconnect_backoff = 0.1
            else:
                self.reconnect_backoff = min(1.0, self.reconnect_backoff * 1.5)
            self.next_reconnect_mono = time.monotonic() + self.reconnect_backoff

        self.close_serial()
        # Emit after fail-closed state/close ownership is established.
        if trace_event is not None:
            self._emit_fault_trace(trace_event)
        safe_reason = json.dumps(str(reason)[:200], ensure_ascii=True)
        self.get_logger().warning(
            f"serial_quarantine state={old_state} epoch={epoch} "
            f"reason={safe_reason} last_state_age_ms={state_age_ms:.1f} "
            f"read_max_ms={read_max_ms:.1f} tx_max_ms={tx_max_ms:.1f} "
            f"worker_gap_max_ms={worker_gap_max_ms:.1f}")


    def _pending_snapshot(self, gen=None):
        """Take a snapshot of pending TX items without clearing."""
        snapshot = []
        now_mono = time.monotonic()
        with self.state_lock:
            if not getattr(self, 'session_ready', False):
                return snapshot

            # Special bootstrap rule: ALLOW one CMD write even if not healthy
            is_healthy = getattr(self, 'telemetry_healthy', False)
            if not is_healthy:
                c_cmd = getattr(self, 'pending_cmd', None)
                if c_cmd:
                    args, ep, ver = c_cmd
                    if ep == getattr(self, 'session_epoch', -1):
                        snapshot.append(('CMD', args, ep, ver, c_cmd))
                return snapshot

            c_cmd = getattr(self, 'pending_cmd', None)
            if c_cmd:
                args, ep, ver = c_cmd
                if ep == getattr(self, 'session_epoch', -1):
                    snapshot.append(('CMD', args, ep, ver, c_cmd))

            # An abandoned ON request must never be refreshed indefinitely.
            last_request = getattr(self, 'last_pump_command_time', None)
            if (getattr(self, 'pump_state', 0) == 1
                    or (self.pending_pump and self.pending_pump[0] == 1)) and (
                    last_request is None or
                    now_mono - last_request > self.pump_command_timeout_s):
                if self.pending_pump is None or self.pending_pump[0] != 0:
                    self.pump_version += 1
                    self.pending_pump = (
                        0, self.session_epoch, self.pump_version)

            c_pump = getattr(self, 'pending_pump', None)
            if c_pump:
                req, ep, ver = c_pump
                if ep == getattr(self, 'session_epoch', -1):
                    snapshot.append(('PUMP', req, ep, ver, c_pump))
            elif now_mono - getattr(self, 'last_pump_time', 0.0) >= 0.5:
                req = getattr(self, 'pump_state', 0)
                ver = getattr(self, 'pump_written_version', 0)
                ep = getattr(self, 'session_epoch', -1)
                snapshot.append(('PUMP_REFRESH', req, ep, ver, None))

            c_fire = getattr(self, 'pending_fire', None)
            if c_fire:
                args, ep, ver = c_fire
                if ep == getattr(self, 'session_epoch', -1):
                    snapshot.append(('FIRE', args, ep, ver, c_fire))

        return snapshot


    def _write_pending_item(self, item, current_ser):
        """Format and write pending item to serial."""
        itype, data, ep, ver, original_tuple = item
        gen = ep
        frame = None
        with self.state_lock:
            if getattr(self, 'ser', None) is not current_ser or self.session_epoch != gen:
                if itype == 'CMD':
                    if getattr(self, 'pending_cmd', None) is original_tuple:
                        self.pending_cmd = None
                        self.cmd_drop_count += 1
                    return True
                if itype == 'PUMP' and data != 0:
                    if getattr(self, 'pending_pump', None) is original_tuple:
                        self.pending_pump = None
                    return True
                if itype == 'FIRE' or itype == 'PUMP_REFRESH':
                    if itype == 'FIRE' and getattr(self, 'pending_fire', None) is original_tuple:
                        self.pending_fire = None
                    return True

            if not getattr(self, 'telemetry_healthy', False):
                if itype == 'CMD' and data != (0.0, 0.0):
                    if getattr(self, 'pending_cmd', None) is original_tuple:
                        self.pending_cmd = None
                        self.cmd_drop_count += 1
                    return True
                if itype == 'PUMP' and data != 0:
                    if getattr(self, 'pending_pump', None) is original_tuple:
                        self.pending_pump = None
                    return True
                if itype == 'FIRE':
                    if getattr(self, 'pending_fire', None) is original_tuple:
                        self.pending_fire = None
                    return True

            guard = getattr(self, '_transport_command_allowed_locked', None)
            if guard is not None and not guard(item):
                self.cmd_drop_count += 1
                return True

            if itype == 'CMD':
                current_item = getattr(self, 'pending_cmd', None)
                if current_item is not original_tuple and current_item is not None:
                    c_args, c_ep, c_ver = current_item
                    if c_ver > ver:
                        return True
                frame = self.protocol.generate_cmd(data[0], data[1])
            elif itype == 'PUMP':
                current_item = getattr(self, 'pending_pump', None)
                if current_item is not original_tuple and current_item is not None:
                    c_req, c_ep, c_ver = current_item
                    if c_ver > ver:
                        return True
                frame = self.protocol.generate_pump(data)
            elif itype == 'PUMP_REFRESH':
                current_item = getattr(self, 'pending_pump', None)
                if current_item is not None:
                    c_req, c_ep, c_ver = current_item
                    if c_ver > ver:
                        return True
                frame = self.protocol.generate_pump(data)
            elif itype == 'FIRE':
                current_item = getattr(self, 'pending_fire', None)
                if current_item is not original_tuple and current_item is not None:
                    c_args, c_ep, c_ver = current_item
                    if c_ver > ver:
                        return True
                frame = self.protocol.generate_fire(data[0], data[1])

        if not frame:
            return True

        if getattr(self, 'stop_request', False):
            return True

        try:
            written = self._timed_write(current_ser, frame)
            if written < len(frame):
                self._increment_counter('tx_partial_count')
                self._quarantine_handle(current_ser, f"tx:{itype}:Exception:Partial write")
                return False
        except Exception as e:
            with self.state_lock:
                if self.ser is current_ser:
                    self.tx_fail_count += 1
            self._quarantine_handle(current_ser, f"tx:{itype}:{type(e).__name__}:{e}")
            return False

        with self.state_lock:
            if itype == 'CMD':
                if getattr(self, 'pending_cmd', None) is original_tuple:
                    self.pending_cmd = None
            elif itype == 'PUMP':
                if getattr(self, 'pending_pump', None) is original_tuple:
                    self.pending_pump = None
                self.pump_state = data
                self.pump_written_version = ver
                self.last_pump_time = time.monotonic()
            elif itype == 'PUMP_REFRESH':
                self.last_pump_time = time.monotonic()
            elif itype == 'FIRE':
                if getattr(self, 'pending_fire', None) is original_tuple:
                    self.pending_fire = None

        return True


    def serial_worker(self):
        """Worker thread for reading and writing."""
        self.get_logger().info("Serial worker thread started.")
        while not getattr(self, 'stop_request', False):
            with self.state_lock:
                state = getattr(self, '_session_state', 'BACKOFF')
                if getattr(self, 'diagnostics_enabled', True):
                    loop_started_mono = time.monotonic()
                    if state in ('WAIT_FIRST_STATE', 'HEALTHY'):
                        previous_tick = self.last_worker_tick_mono
                        if previous_tick is not None:
                            self.worker_gap_max_ms = max(
                                self.worker_gap_max_ms,
                                max(0.0, (loop_started_mono - previous_tick)
                                    * 1000.0))
                        self.last_worker_tick_mono = loop_started_mono
                    else:
                        self.last_worker_tick_mono = None
                exact_handle = self.ser
                gen = self.session_epoch
                has_close_task = self.close_ownership.get('task') is not None

            if state == 'SHUTTING_DOWN':
                break

            if state == 'CLOSING' or has_close_task:
                res = self.close_serial()
                if res != 'CLOSED':
                    time.sleep(0.01)
                else:
                    with self.state_lock:
                        if getattr(self, '_session_state', '') == 'CLOSING':
                            self._transition_state('BACKOFF')
                continue

            if state == 'BACKOFF':
                now = time.monotonic()
                if now < getattr(self, 'next_reconnect_mono', 0.0):
                    time.sleep(0.01)
                    continue
                with self.state_lock:
                    if (getattr(self, 'stop_request', False)
                            or getattr(self, '_session_state', '')
                            == 'SHUTTING_DOWN'):
                        break
                    self.session_epoch += 1
                    self.open_generation = self.session_epoch
                    self._transition_state('OPENING')
                continue

            if state == 'OPENING':
                self._increment_counter('open_attempt_count')
                with self.state_lock:
                    trace = self._fault_trace_pending
                    trace_open = (trace is not None
                                  and trace.get('open_attempts', 0) < 4)
                    if trace_open:
                        trace['open_attempts'] = trace.get(
                            'open_attempts', 0) + 1
                        trace_id = trace['id']
                    else:
                        trace_id = None
                open_start_ns = time.monotonic_ns() if trace_open else None
                try:
                    new_ser = self.serial_cls(
                        self.port, self.baudrate,
                        timeout=0.01, write_timeout=0.1)
                except Exception as e:
                    if trace_open:
                        self._emit_fault_trace({
                            'phase': 'open_failed', 'id': trace_id,
                            'epoch': gen, 'wall_ns': time.time_ns(),
                            'duration_ms': round((time.monotonic_ns()
                                                  - open_start_ns) / 1e6, 3),
                            'error': type(e).__name__,
                        })
                    self.get_logger().warn(
                        f"Failed to open port {self.port}: {e}")
                    with self.state_lock:
                        self.open_fail_count += 1
                        self.last_failure_reason = (
                            f"open:{type(e).__name__}:{e}")
                        self.reconnect_backoff = min(
                            1.0, max(0.1, self.reconnect_backoff * 1.5))
                        self.next_reconnect_mono = (
                            time.monotonic() + self.reconnect_backoff)
                        self._transition_state('BACKOFF')
                    continue

                open_end_ns = time.monotonic_ns() if trace_open else None
                try:
                    cmd = self.protocol.generate_cmd(0.0, 0.0)
                    pump = self.protocol.generate_pump(0)

                    if cmd:
                        w1 = self._timed_write(new_ser, cmd)
                        if w1 < len(cmd):
                            raise Exception("Partial write during bootstrap (CMD)")
                    if getattr(self, 'stop_request', False):
                        raise Exception("Bootstrap interrupted by shutdown")
                    if pump:
                        w2 = self._timed_write(new_ser, pump)
                        if w2 < len(pump):
                            raise Exception("Partial write during bootstrap (PUMP)")
                    if getattr(self, 'stop_request', False):
                        raise Exception("Bootstrap interrupted by shutdown")
                except Exception as e:
                    if trace_open:
                        self._emit_fault_trace({
                            'phase': 'bootstrap_failed', 'id': trace_id,
                            'epoch': gen, 'wall_ns': time.time_ns(),
                            'open_ms': round((open_end_ns - open_start_ns)
                                             / 1e6, 3),
                            'bootstrap_ms': round((time.monotonic_ns()
                                                   - open_end_ns) / 1e6, 3),
                            'error': type(e).__name__,
                        })
                    self._increment_counter('bootstrap_fail_count')
                    self._quarantine_handle(
                        new_ser,
                        f"bootstrap:{type(e).__name__}:{e}")
                    continue

                bootstrap_end_ns = time.monotonic_ns() if trace_open else None

                reject_reason = None
                with self.state_lock:
                    if (getattr(self, 'stop_request', False)
                            or getattr(self, '_session_state', '')
                            in ('CLOSING', 'SHUTTING_DOWN')
                            or self.session_epoch
                            != getattr(self, 'open_generation', -1)
                            or getattr(self, 'ser', None) is not None
                            or self.close_ownership.get('handle')
                            is not None
                            or self.close_ownership['pending']):
                        reject_reason = 'bootstrap:Abort:State changed'
                    else:
                        now = time.monotonic()
                        self.open_success_count += 1
                        self.reconnect_total += 1
                        self.reconnect_count += 1
                        self.ser = new_ser
                        self.session_ready = True
                        self.telemetry_healthy = False
                        self.session_started_mono = now
                        self.last_state_time_mono = 0.0
                        if self._fault_read_window is not None:
                            self._fault_read_window.clear()
                            self._fault_last_nonempty_read_ns = None
                        self.last_pump_time = now
                        self.pump_state = 0
                        self.last_pump_command_time = None
                        self._transition_state('WAIT_FIRST_STATE')

                if reject_reason is not None:
                    self._quarantine_handle(new_ser, reject_reason)
                    continue

                self.protocol.reset_parser()
                trace_event = None
                with self.state_lock:
                    last_quarantine = self.last_quarantine_mono
                    recovery_ms = (
                        max(0.0, (time.monotonic() - last_quarantine) * 1000.0)
                        if last_quarantine is not None else -1.0)
                    opened_epoch = self.session_epoch
                    trace = self._fault_trace_pending
                    if (trace is not None and trace['id'] == trace_id
                            and bootstrap_end_ns is not None):
                        trace_event = {
                            'phase': 'opened', 'id': trace['id'],
                            'epoch': opened_epoch,
                            'open_start_mono_ns': open_start_ns,
                            'open_end_mono_ns': open_end_ns,
                            'bootstrap_end_mono_ns': bootstrap_end_ns,
                            'open_ms': round((open_end_ns - open_start_ns) / 1e6, 3),
                            'bootstrap_ms': round(
                                (bootstrap_end_ns - open_end_ns) / 1e6, 3),
                            'fault_to_open_ms': round(
                                (bootstrap_end_ns - trace['fault_mono_ns']) / 1e6, 3),
                        }
                if trace_event is not None:
                    trace_event['opened_wall_ns'] = time.time_ns()
                    self._emit_fault_trace(trace_event)
                self.get_logger().info(
                    f"Opened serial port {self.port} epoch={opened_epoch} "
                    f"recovery_ms={recovery_ms:.1f}")

                continue

            if exact_handle is None:
                time.sleep(0.01)
                continue

            try:
                data = self._read_with_fault_trace(exact_handle, 1024, gen)
                if data:
                    for frame in self.protocol.parse_chunk(data):
                        self.handle_frame(frame)
            except Exception as e:
                self._increment_counter('read_fail_count')
                self._quarantine_handle(exact_handle, f"read:{type(e).__name__}:{e}")
                continue

            items = self._pending_snapshot(gen)
            if items:
                tx_acquired = self.tx_lock.acquire(timeout=0.01)
                if tx_acquired:
                    try:
                        for item in items:
                            res = self._write_pending_item(item, exact_handle)
                            if not res:
                                # exact_handle is already quarantined by _write_pending_item!
                                break
                    finally:
                        self.tx_lock.release()
            else:
                time.sleep(0.005)

            timeout_reason = None
            now = time.monotonic()
            with self.state_lock:
                same_session = (
                    self.ser is exact_handle and self.session_epoch == gen)
                current_state = self._session_state
                if (same_session and current_state == 'WAIT_FIRST_STATE'
                        and now - self.session_started_mono
                        > self.first_state_timeout_s):
                    self.first_state_timeout_count += 1
                    timeout_reason = 'first_state:Timeout'
                elif (same_session and current_state == 'HEALTHY'
                      and now - self.last_state_time_mono > 0.5):
                    timeout_reason = 'healthy:Timeout'
            if timeout_reason is not None:
                self._quarantine_handle(exact_handle, timeout_reason)

        self.get_logger().info("Serial worker thread exited.")


    def handle_frame(self, frame):
        """Handle parsed telemetry frame."""
        now_mono = time.monotonic()
        trace_event = None
        boot_event = None
        usb_recovery_event = None
        with self.state_lock:
            if not self.session_ready:
                return
            epoch = self.session_epoch

            if frame['type'] == 'STATE':
                if (self.fault_trace_events
                        and (self.fault_trace_count < self.fault_trace_events
                             or self._fault_trace_pending is not None)):
                    current = {
                        'epoch': epoch,
                        'esp_ms': frame['esp_ms'],
                        'receive_mono_ns': int(now_mono * 1_000_000_000),
                        'pi_wall_ns': time.time_ns(),
                    }
                    trace = self._fault_trace_pending
                    if trace is not None and epoch != trace['fault_epoch']:
                        before = trace['last_state']
                        trace_event = {
                            'phase': 'first_state', 'id': trace['id'],
                            'before': before, 'after': current,
                            'uptime_evidence': classify_esp_uptime(
                                before, current),
                        }
                        self._fault_trace_pending = None
                        if self.fault_trace_count >= self.fault_trace_events:
                            self._fault_read_window = None
                            self._fault_last_nonempty_read_ns = None
                    self._fault_trace_last_state = current
                if (getattr(self, 'diagnostics_enabled', True)
                        and self.last_state_time_mono > 0.0):
                    self.state_gap_max_ms = max(
                        self.state_gap_max_ms,
                        max(0.0, (now_mono - self.last_state_time_mono) * 1000.0))
                self.last_state_time_mono = now_mono
                self.telemetry_healthy = True
                self.reconnect_backoff = 0.0
                self.next_reconnect_mono = 0.0
                if (self.latest_state is not None
                        and getattr(self, 'diagnostics_enabled', True)):
                    self.odom_timing.increment('state_slot_overwrites')
                self.latest_state = (frame, epoch, now_mono)
                self._transition_state('HEALTHY')
            elif frame['type'] == 'ENV':
                self.last_env_time_mono = now_mono
                self.latest_env = (frame, epoch, now_mono)
            elif frame['type'] == 'IR':
                self.last_ir_time_mono = now_mono
                self.latest_ir = (frame, epoch, now_mono)
            elif frame['type'] == 'ACT':
                self.latest_actuator = (frame, epoch, now_mono)
            elif frame['type'] == 'USB':
                # Do not refresh STATE time, safety readiness or reconnect state.
                self.latest_usb_diagnostic = (frame, epoch, now_mono)
                recoveries = frame.get('recoveries')
                if recoveries is not None:
                    previous = self._usb_recoveries_seen
                    self._usb_recoveries_seen = recoveries
                    if (recoveries > 0 and
                            (previous is None or recoveries > previous) and
                            self._usb_recovery_events_logged < 8):
                        self._usb_recovery_events_logged += 1
                        usb_recovery_event = {
                            'epoch': epoch,
                            'kind': ('before_first_usb_frame' if previous is None
                                     else 'counter_advanced'),
                            'previous_recoveries': previous,
                            'pi_wall_ns': time.time_ns(),
                            'receive_mono_ns': int(now_mono * 1e9),
                            'usb': frame,
                        }
            elif frame['type'] == 'BOOT':
                # Child process log is sufficient; no IPC telemetry slot or
                # timer. This diagnostic never affects command admission.
                if self._boot_logged_epoch != epoch:
                    self._boot_logged_epoch = epoch
                    boot_event = {
                        'epoch': epoch, 'receive_mono_ns': int(now_mono * 1e9),
                        'pi_wall_ns': time.time_ns(), 'esp_ms': frame['esp_ms'],
                        'reset_reason': frame['reset_reason'],
                        'elf_sha256': frame['elf_sha256'],
                    }

        if trace_event is not None:
            self._emit_fault_trace(trace_event)
        if boot_event is not None:
            self.get_logger().info('esp_boot_identity ' + json.dumps(
                boot_event, sort_keys=True, separators=(',', ':')))
        if usb_recovery_event is not None:
            self.get_logger().warning('esp_usb_recovery_event ' + json.dumps(
                usb_recovery_event, sort_keys=True, separators=(',', ':')))


    def log_timer_callback(self):
        """Log diagnostics."""
        with getattr(self, 'state_lock', threading.Lock()):
            state = getattr(self, '_session_state', 'BACKOFF')
            drop = getattr(self, 'cmd_drop_count', 0)
            self.cmd_drop_count = 0
            t_drop = getattr(self, 'telemetry_drop_count', 0)
            self.telemetry_drop_count = 0
            t_pub = getattr(self, 'telemetry_publish_count', 0)
            self.telemetry_publish_count = 0
            t_fail = getattr(self, 'telemetry_publish_fail_count', 0)
            self.telemetry_publish_fail_count = 0
            tx_p = getattr(self, 'tx_partial_count', 0)
            self.tx_partial_count = 0
            tx_f = getattr(self, 'tx_fail_count', 0)
            self.tx_fail_count = 0
            s_max = getattr(self, 'state_publish_max_age', 0.0)
            self.state_publish_max_age = 0.0
            e_max = getattr(self, 'env_publish_max_age', 0.0)
            self.env_publish_max_age = 0.0
            read_max_ms = getattr(self, 'read_max_ms', 0.0)
            self.read_max_ms = 0.0
            tx_max_ms = getattr(self, 'tx_max_ms', 0.0)
            self.tx_max_ms = 0.0
            worker_gap_max_ms = getattr(self, 'worker_gap_max_ms', 0.0)
            self.worker_gap_max_ms = 0.0
            state_gap_max_ms = getattr(self, 'state_gap_max_ms', 0.0)
            self.state_gap_max_ms = 0.0

            open_attempt = getattr(self, 'open_attempt_count', 0)
            open_succ = getattr(self, 'open_success_count', 0)
            open_fail = getattr(self, 'open_fail_count', 0)
            first_state_timeout = getattr(self, 'first_state_timeout_count', 0)
            recon = getattr(self, 'reconnect_count', 0)
            self.reconnect_count = 0
            recon_total = getattr(self, 'reconnect_total', 0)
            read_f = getattr(self, 'read_fail_count', 0)
            boot_f = getattr(self, 'bootstrap_fail_count', 0)
            close_f = getattr(self, 'close_fail_count', 0)

        self.get_logger().info(
            f"state={state} "
            f"open_attempt_count={open_attempt} "
            f"open_success_count={open_succ} "
            f"open_fail_count={open_fail} "
            f"first_state_timeout_count={first_state_timeout} "
            f"reconnect_count={recon} "
            f"reconnect_total={recon_total} "
            f"read_fail_count={read_f} "
            f"bootstrap_fail_count={boot_f} "
            f"close_fail_count={close_f} "
            f"cmd_drop_count={drop} "
            f"telemetry_drop_count={t_drop} "
            f"telemetry_publish_count={t_pub} "
            f"telemetry_publish_fail_count={t_fail} "
            f"tx_partial_count={tx_p} "
            f"tx_fail_count={tx_f} "
            f"state_publish_max_age={s_max:.3f} "
            f"env_publish_max_age={e_max:.3f} "
            f"read_max_ms={read_max_ms:.1f} "
            f"tx_max_ms={tx_max_ms:.1f} "
            f"worker_gap_max_ms={worker_gap_max_ms:.1f} "
            f"state_gap_max_ms={state_gap_max_ms:.1f} "
            f"odom_timing=[{self.odom_timing.format_snapshot()}]"
        )


    def shutdown_transport(self):
        """Shutdown the node safely."""
        if getattr(self, 'node_destroyed', False):
            return True

        timers_to_cancel = []
        with getattr(self, 'state_lock', threading.Lock()):
            if not getattr(self, '_shutdown_started', False):
                self._shutdown_started = True
                self._shutdown_deadline_mono = time.monotonic() + 3.5
                self._transition_state('SHUTTING_DOWN')
                self.stop_request = True
                self.running = False
                self.session_ready = False
                self.telemetry_healthy = False
                self.session_epoch += 1
                self.pending_cmd = None
                self.pending_pump = None
                self.pending_fire = None
                self.latest_state = None
                self.latest_env = None
                self.latest_ir = None
                self.latest_actuator = None
                self.session_started_mono = 0.0
                self.last_state_time_mono = 0.0
                self.close_ownership['deadline'] = (
                    self._shutdown_deadline_mono)
                timers_to_cancel = [
                    timer for timer in (
                        getattr(self, 'telemetry_timer', None),
                        getattr(self, 'log_timer', None))
                    if timer is not None]

        for timer in timers_to_cancel:
            try:
                timer.cancel()
            except Exception as e:
                self.get_logger().warn(f"Failed to cancel timer: {e}")

        deadline = getattr(self, '_shutdown_deadline_mono', time.monotonic())

        needs_close = False
        with self.state_lock:
            if (getattr(self, 'ser', None) is not None
                    or self.close_ownership.get('handle') is not None
                    or self.close_ownership.get('pending')):
                needs_close = True

        res = 'CLOSED'
        if needs_close:
            res = self.close_serial()
            if res != 'CLOSED':
                task = self.close_ownership.get('task')
                if task is not None and task is not threading.current_thread():
                    remaining = max(0.0, deadline - time.monotonic())
                    task.join(timeout=min(0.05, remaining))
                res = self.close_serial()
                if res != 'CLOSED':
                    return False

        if hasattr(self, 'worker_thread') and self.worker_thread.is_alive():
            if self.worker_thread is not threading.current_thread():
                remaining = max(0.0, deadline - time.monotonic())
                self.worker_thread.join(timeout=remaining)
                if self.worker_thread.is_alive():
                    if time.monotonic() >= deadline:
                        self.get_logger().error(
                            "Teardown budget exhausted, failing closed (worker thread).")
                    return False

        pl = getattr(self, 'publish_lock', None)
        if pl:
            remaining = max(0.0, deadline - time.monotonic())
            if not pl.acquire(timeout=remaining):
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        "Teardown budget exhausted, failing closed (publish_lock).")
                return False
            pl.release()

        tl = getattr(self, 'tx_lock', None)
        if tl:
            remaining = max(0.0, deadline - time.monotonic())
            if not tl.acquire(timeout=remaining):
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        "Teardown budget exhausted, failing closed (tx_lock).")
                return False
            tl.release()

        return True
