"""Private FD-based bootstrap for a candidate operator process.

The secret is transferred through an already-open anonymous file descriptor,
not argv, environment or a public file path. The descriptor contents bind the
operator process to its frontend session; this does not verify the R5 packet.
"""
import base64
import binascii
import fcntl
import os
from pathlib import Path
import stat
import tempfile

from .operator_runner import OperatorBootstrap


MAX_BOOTSTRAP_BYTES = 4096
def create_bootstrap_file(bootstrap, runtime_dir):
    """Return an anonymous private file; caller passes its FD to the child."""
    if not isinstance(bootstrap, OperatorBootstrap):
        raise TypeError("OperatorBootstrap required")
    bootstrap.validate()
    directory = Path(runtime_dir)
    info = directory.lstat()
    if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid()
            or stat.S_IMODE(info.st_mode) & 0o077):
        raise ValueError("private runtime directory required")
    # Fixed ordered lines have no duplicate keys or JSON parser differences
    # between Python and Qt; encode the path to keep newlines unambiguous.
    path = str(bootstrap.socket_path).encode("utf-8")
    payload = b"\n".join((b"1", bootstrap.session.encode("ascii"),
        bootstrap.generation.encode("ascii"), bootstrap.nonce.encode("ascii"),
        bootstrap.packet.encode("ascii"), base64.b64encode(path), b""))
    if len(payload) > MAX_BOOTSTRAP_BYTES:
        raise ValueError("bootstrap too large")
    file = tempfile.TemporaryFile(mode="w+b", dir=directory)
    try:
        os.fchmod(file.fileno(), 0o600)
        file.write(payload)
        file.flush()
        # Hand the child a read-only descriptor rather than the writer used
        # to assemble the bootstrap. TemporaryFile remains anonymous.
        fd = os.open(f"/proc/self/fd/{file.fileno()}", os.O_RDONLY | os.O_CLOEXEC)
        file.close()
        try:
            return os.fdopen(fd, "rb")
        except BaseException:
            os.close(fd)
            raise
    except BaseException:
        file.close()
        raise


def read_bootstrap_fd(fd):
    """Read once with pread so multiple inherited FD offsets do not interfere."""
    if type(fd) is not int or fd < 3:
        raise ValueError("invalid bootstrap descriptor")
    try:
        info = os.fstat(fd)
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                or stat.S_IMODE(info.st_mode) & 0o077 or not 0 < info.st_size <= MAX_BOOTSTRAP_BYTES
                or fcntl.fcntl(fd, fcntl.F_GETFL) & os.O_ACCMODE != os.O_RDONLY):
            raise ValueError("unsafe bootstrap descriptor")
        payload = os.pread(fd, MAX_BOOTSTRAP_BYTES + 1, 0)
        if len(payload) != info.st_size:
            raise ValueError("bootstrap changed while reading")
        parts = payload.split(b"\n")
        if len(parts) != 7 or parts[0] != b"1" or parts[-1] != b"":
            raise ValueError("invalid bootstrap fields")
        decoded_path = base64.b64decode(parts[5], validate=True)
        if base64.b64encode(decoded_path) != parts[5] or b"\0" in decoded_path:
            raise ValueError("invalid socket path")
        result = OperatorBootstrap(parts[1].decode("ascii"), parts[2].decode("ascii"),
                                   parts[3].decode("ascii"), parts[4].decode("ascii"),
                                   Path(decoded_path.decode("utf-8")))
        result.validate()
        return result
    except (OSError, TypeError, UnicodeDecodeError, RecursionError, binascii.Error) as exc:
        raise ValueError("cannot read bootstrap") from exc
