"""Nonblocking PTY transport for a future owned runner, not a STOP mechanism.

This module starts no process on import. The caller must choose a vetted argv,
protect interactive input, keep the Pi runner alive for the laptop drain, and
use the R5 receipts for STOP/cleanup. The PTY's exit code is not robot proof.
"""
import errno
import fcntl
import os
import pty
import struct
import subprocess
import termios


class PtyChild:
    def __init__(self, argv, *, env=None, cwd=None):
        if (not isinstance(argv, (list, tuple)) or not argv
                or any(not isinstance(arg, str) or not arg or "\0" in arg for arg in argv)):
            raise ValueError("argv must be a non-empty sequence of strings")
        master, slave = pty.openpty()
        self.master = master
        self.process = None
        try:
            attrs = termios.tcgetattr(slave)
            attrs[3] &= ~termios.ECHO
            termios.tcsetattr(slave, termios.TCSANOW, attrs)
            fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack("HHHH", 40, 120, 0, 0))
            os.set_blocking(master, False)
            # setsid -c acquires the slave as controlling TTY without unsafe
            # Python preexec_fn in a process that may already have threads.
            self.process = subprocess.Popen(
                ["/usr/bin/setsid", "--ctty", "--wait", *argv],
                stdin=slave, stdout=slave, stderr=slave,
                env=env, cwd=cwd, close_fds=True)
        except BaseException:
            os.close(master)
            self.master = None
            raise
        finally:
            os.close(slave)

    def read(self, maximum=8192):
        """Return available raw bytes; caller must redact and bound logging.

        PTY output may contain secrets if a child changes echo settings. Never
        display or store this stream without an independently reviewed filter.
        Empty bytes mean no data or EOF; inspect poll() separately for exit.
        """
        if type(maximum) is not int or not 0 < maximum <= 8192:
            raise ValueError("invalid PTY read size")
        if self.master is None:
            return b""
        try:
            return os.read(self.master, maximum)
        except BlockingIOError:
            return b""
        except OSError as exc:
            if exc.errno == errno.EIO:
                return b""
            raise

    def write(self, data):
        """Write input without logging; return bytes accepted, possibly zero."""
        if not isinstance(data, bytes) or len(data) > 4096:
            raise ValueError("invalid PTY input")
        if self.master is None or self.process.poll() is not None:
            raise BrokenPipeError("runner has exited")
        try:
            return os.write(self.master, data)
        except BlockingIOError:
            return 0

    def poll(self):
        return self.process.poll()

    def close_after_exit(self):
        """Close the descriptor only after exit; never use this to stop Pi."""
        if self.process.poll() is None:
            raise RuntimeError("runner still active; STOP/drain must run first")
        if self.master is not None:
            os.close(self.master)
            self.master = None
