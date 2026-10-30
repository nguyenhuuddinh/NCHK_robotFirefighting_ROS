"""Private pairing download for a future owned Pi runner.

No process or network operation occurs on import. The caller must provide a
real ownership check and keep this object alive until the laptop runner has
finished reading the file. The current desktop app does not invoke this code.
"""
import os
from pathlib import Path
import stat
import tempfile

from .pairing import MAX_LINE, PairingAnnouncement, PairingError, PairingOutputParser, validate_binding
from .pty_child import PtyChild


class PairingTransferError(RuntimeError):
    pass


class PairingTransfer:
    def __init__(self, announcement, packet, *, host, runtime_dir, assert_owned_pi,
                 child_factory=PtyChild):
        if host not in ("10.0.0.1", "10.42.0.185") or not isinstance(announcement, PairingAnnouncement):
            raise PairingTransferError("Địa chỉ Pi hoặc pairing không hợp lệ")
        if not callable(assert_owned_pi):
            raise TypeError("Pi ownership proof required")
        # Reuse exactly the same strict path grammar as the current Pi output.
        parser = PairingOutputParser()
        try:
            output = ("LOG_PI=" + announcement.log_dir + "\nPI_PAIRING=" +
                      announcement.pairing_path + "\n").encode("utf-8")
            if parser.feed(output) != [announcement]:
                raise PairingError("invalid announcement")
        except (PairingError, UnicodeEncodeError, TypeError) as exc:
            raise PairingTransferError("Đường dẫn pairing không hợp lệ") from exc
        if not isinstance(packet, str) or len(packet) != 64 or any(c not in "0123456789abcdef" for c in packet):
            raise PairingTransferError("Packet không hợp lệ")
        runtime = Path(runtime_dir)
        info = runtime.lstat()
        if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid()
                or stat.S_IMODE(info.st_mode) & 0o077):
            raise PairingTransferError("Thư mục phiên không riêng tư")
        assert_owned_pi()
        self.announcement, self.packet = announcement, packet
        self.assert_owned_pi = assert_owned_pi
        self.temp = tempfile.TemporaryDirectory(prefix="pairing-", dir=runtime)
        self.directory = Path(self.temp.name)
        self.path = self.directory / "pairing.json"
        self.finished = False
        self.child = None
        try:
            fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
            try:
                identity = os.fstat(fd)
                self.identity = (identity.st_dev, identity.st_ino)
            finally:
                os.close(fd)
            # Host and remote path are strictly validated above; no shell is used.
            self.child = child_factory(["scp", "-q", "-o", "ConnectTimeout=8",
                "pi@" + host + ":" + announcement.pairing_path, str(self.path)])
        except BaseException:
            self.temp.cleanup()
            raise

    def finish(self):
        """Return a private file path only after the transfer and binding pass."""
        if self.finished or self.child is None or self.child.poll() is None:
            raise PairingTransferError("Chưa thể xác nhận pairing")
        self.finished = True
        if self.child.poll() != 0:
            raise PairingTransferError("Không tải được pairing từ Pi")
        self.assert_owned_pi()
        try:
            fd = os.open(self.path, os.O_RDONLY | os.O_NOFOLLOW)
            try:
                info = os.fstat(fd)
                if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                        or stat.S_IMODE(info.st_mode) & 0o077
                        or (info.st_dev, info.st_ino) != self.identity):
                    raise PairingTransferError("File pairing không còn thuộc phiên này")
                data = os.read(fd, MAX_LINE + 1)
            finally:
                os.close(fd)
            validate_binding(data, self.announcement, self.packet)
            self.assert_owned_pi()
        except (OSError, PairingError) as exc:
            raise PairingTransferError("Pairing sai hoặc không thể đọc an toàn") from exc
        return self.path

    def close_after_exit(self):
        """The caller must retain the file through the laptop's complete use."""
        if self.child is not None:
            if self.child.poll() is None:
                raise PairingTransferError("Transfer còn chạy")
            self.child.close_after_exit()
        self.temp.cleanup()
