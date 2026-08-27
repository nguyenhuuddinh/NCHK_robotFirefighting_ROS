# Upstream Provenance for serial package

**Source**: https://github.com/ZhaoXiangBox/serial.git
**Base Commit**: `aef041823bc82786249f17c3789d24b15f32e8b9`
**License**: MIT (extracted from `include/serial/serial.h`)

This package is vendored directly into the `ros2_ws/src` tree as an ordinary source directory to satisfy dependency requirements without relying on unstable submodules or external remote dependencies that may change.

## Applied Patches
### L10-2: Node Destructor Tearing & Exception-Safe Close
**File affected:** `src/impl/unix.cc`

**Modifications:**
1. **`Serial::SerialImpl::close()`**: The file descriptor (`fd_`) and `is_open_` states are guaranteed to be transitioned to their finalized state (`-1` and `false`) *before* any `IOException` is thrown when the `close()` system call fails.
2. **`Serial::SerialImpl::~SerialImpl()`**: The `close()` call is wrapped in a `try-catch` block, preventing exceptions thrown by `close()` from escaping the destructor, which would normally invoke `std::terminate()`.

These changes were made to fix a teardown bug where a failed explicit close would cause the destructor to retry closing the stale file descriptor, resulting in process termination.

### Testing and Policy
This library is treated as a third-party legacy package.
- It is NOT subjected to aggressive mass-formatting (`cpplint`, `uncrustify`) as doing so would alter thousands of lines of upstream code and break future patch capability.
- `ament_cppcheck` and `ament_lint_cmake` are preserved and expected to pass.
- Regression testing (ASan/UBSan, Unix FD exception safety) is managed externally in the `camsense_x1` test suite.
