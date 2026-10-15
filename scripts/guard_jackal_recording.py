#!/usr/bin/env python3
"""Linux exec wrapper: stop rosbag gracefully at a free-space or time limit."""
import argparse
import fcntl
import math
import os
from pathlib import Path
import select
import signal
import sys
import time

GIB = 1024 ** 3


def available_bytes(path):
    stats = os.statvfs(path)
    # Space available to the recording user, excluding root's reserved blocks.
    return stats.f_bavail * stats.f_frsize


def stop_reason(free_bytes, minimum, elapsed, maximum):
    if free_bytes <= minimum:
        return f"Free space {free_bytes / GIB:.2f} GiB reached reserve {minimum / GIB:g} GiB"
    if maximum and elapsed >= maximum:
        return f"Session time limit reached ({maximum:g} seconds, including pauses)"
    return None


def watch(parent_pid, ready_fd, path, minimum, maximum, interval):
    # A pidfd binds monitoring/signalling to this exact recorder process even
    # after exec, and prevents signalling an unrelated process if a PID is reused.
    parent_fd = os.pidfd_open(parent_pid)
    try:
        started = time.monotonic()
        os.write(ready_fd, b"R")
        os.close(ready_fd)
        while not select.select([parent_fd], [], [], interval)[0]:
            try:
                reason = stop_reason(
                    available_bytes(path), minimum,
                    time.monotonic() - started, maximum)
            except OSError as error:
                reason = f"Cannot check recording filesystem: {error}"
            if reason:
                print(f"\n[storage guard] {reason}. Sending SIGINT; waiting for bag flush. "
                      "No files will be deleted.", flush=True)
                try:
                    signal.pidfd_send_signal(parent_fd, signal.SIGINT)
                except ProcessLookupError:
                    return
                # Do not escalate to SIGKILL: let rosbag drain its cache/metadata.
                while not select.select([parent_fd], [], [], 1)[0]:
                    pass
                return
    except BaseException:
        try:
            signal.pidfd_send_signal(parent_fd, signal.SIGINT)
        except ProcessLookupError:
            pass
        raise
    finally:
        os.close(parent_fd)


def launch_watchdog(path, lock_fd, minimum, maximum, interval):
    read_fd, ready_fd = os.pipe()
    parent_pid = os.getpid()
    child_pid = os.fork()
    if child_pid == 0:
        os.close(read_fd)
        os.close(lock_fd)  # Only the recorder should keep the single-session lock.
        os.setsid()  # Terminal Ctrl+C must reach rosbag, not its disk monitor.
        try:
            watch(parent_pid, ready_fd, path, minimum, maximum, interval)
        except BaseException as error:
            print(f"[storage guard] Monitor failed: {error}", file=sys.stderr, flush=True)
            # Startup failure is rejected by the ready handshake.
        finally:
            os._exit(0)
    os.close(ready_fd)
    try:
        if not select.select([read_fd], [], [], 5)[0] or os.read(read_fd, 1) != b"R":
            raise RuntimeError("Storage monitor did not start; recording refused")
    except BaseException:
        try:
            os.kill(child_pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        os.waitpid(child_pid, 0)
        raise
    finally:
        os.close(read_fd)


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def nonnegative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("must be a finite number >= 0")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--min-free-gib", type=positive, default=100)
    parser.add_argument("--max-seconds", type=nonnegative, default=3600,
                        help="wall time after first Space, including pauses; 0 disables")
    parser.add_argument("--check-seconds", type=positive, default=5)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command and command[0] == "--":
        command = command[1:]
    if not args.check_only and not command:
        parser.error("a recorder command after -- is required")
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        parser.error("Linux with Python pidfd support is required")
    output = args.output.expanduser().absolute()
    if output.exists() or output.is_symlink():
        raise ValueError(f"Output already exists; choose a new directory: {output}")
    parent = output.parent
    parent.mkdir(parents=True, exist_ok=True)
    minimum = int(args.min_free_gib * GIB)
    free = available_bytes(parent)
    print(f"Storage: {parent}\nAvailable: {free / GIB:.2f} GiB; "
          f"reserve: {args.min_free_gib:g} GiB", flush=True)
    if free <= minimum:
        raise ValueError("Insufficient free space: transfer and verify old bags, or manually "
                         "delete unneeded data, before recording. No files were deleted.")
    # Lock per recording parent directory; survives exec, released on recorder exit.
    lock_fd = os.open(parent / ".record_jackal.lock", os.O_RDWR | os.O_CREAT, 0o600)
    try:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError(f"Another guarded recorder is active in {parent}") from None
        if args.check_only:
            return 0
        os.set_inheritable(lock_fd, True)
        print(f"Storage guard: check every {args.check_seconds:g}s; "
              f"session limit: {args.max_seconds:g}s (0 = unlimited).", flush=True)
        launch_watchdog(parent, lock_fd, minimum, args.max_seconds, args.check_seconds)
        # Keep this PID and foreground terminal: native Space/Ctrl+C still work.
        os.execvp(command[0], command)
    finally:
        os.close(lock_fd)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(130)
