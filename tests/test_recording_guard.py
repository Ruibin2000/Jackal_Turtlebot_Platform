"""Exercise disk refusal, graceful shutdown, locks and terminal controls without ROS."""
import importlib.util
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import time
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/guard_jackal_recording.py"
spec = importlib.util.spec_from_file_location("guard", SCRIPT)
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


class RecordingGuardTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.fake = self.root / "fake_recorder.py"
        self.fake.write_text("""
import os, signal, sys, time
from pathlib import Path
root = Path(sys.argv[1])
def stop(signum, frame):
    time.sleep(0.1)  # Model final cache/metadata flush.
    (root / 'saved').write_text('graceful')
    raise SystemExit(0)
signal.signal(signal.SIGINT, stop)
if os.isatty(0):
    import tty
    tty.setcbreak(0)
(root / 'started').touch()
if len(sys.argv) > 2:
    Path(sys.argv[2]).write_text('0')
while True:
    if os.isatty(0):
        key = os.read(0, 1)
        if key == b' ':
            (root / 'space').touch()
    else:
        time.sleep(0.02)
""")

    def args(self, maximum="0.3", output="bag"):
        return ["--output", str(self.root / output), "--min-free-gib", "0.000001",
                "--max-seconds", maximum, "--check-seconds", "0.05"]

    def wait_file(self, name, timeout=4):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            if (self.root / name).exists():
                return
            time.sleep(0.02)
        self.fail(f"No {name} marker")

    def test_low_space_refuses_without_starting_or_deleting(self):
        old = self.root / "old.mcap"
        old.write_text("keep")
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--output", str(self.root / "bag"),
             "--min-free-gib", "1000000000", "--",
             sys.executable, str(self.fake), str(self.root)],
            capture_output=True, text=True, timeout=5)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("transfer and verify", result.stderr)
        self.assertFalse((self.root / "started").exists())
        self.assertFalse((self.root / "bag").exists())
        self.assertEqual(old.read_text(), "keep")

    def test_timer_sends_sigint_and_allows_final_flush(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), *self.args(), "--",
             sys.executable, str(self.fake), str(self.root)],
            capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((self.root / "saved").read_text(), "graceful")
        self.assertIn("Session time limit", result.stdout)

    def test_space_drop_during_recording_stops_gracefully(self):
        free = self.root / "free"
        free.write_text(str(guard.GIB))
        launcher = (
            "import importlib.util, pathlib; "
            f"s=importlib.util.spec_from_file_location('guard', {str(SCRIPT)!r}); "
            "m=importlib.util.module_from_spec(s); s.loader.exec_module(m); "
            f"m.available_bytes=lambda path: int(pathlib.Path({str(free)!r}).read_text()); "
            "m.main()")
        result = subprocess.run(
            [sys.executable, "-c", launcher, *self.args("0"), "--",
             sys.executable, str(self.fake), str(self.root), str(free)],
            capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue((self.root / "saved").exists())
        self.assertIn("reached reserve", result.stdout)

    def test_lock_blocks_second_recording_and_releases_on_exit(self):
        proc = subprocess.Popen(
            [sys.executable, str(SCRIPT), *self.args("0"), "--",
             sys.executable, str(self.fake), str(self.root)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            self.wait_file("started")
            check = [sys.executable, str(SCRIPT), *self.args(output="bag2"), "--check-only"]
            denied = subprocess.run(check, capture_output=True, text=True, timeout=3)
            self.assertNotEqual(denied.returncode, 0)
            self.assertIn("Another guarded recorder", denied.stderr)
            proc.send_signal(signal.SIGINT)
            proc.communicate(timeout=5)
            allowed = subprocess.run(check, capture_output=True, text=True, timeout=3)
            self.assertEqual(allowed.returncode, 0, allowed.stderr)
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                proc.communicate(timeout=5)

    def test_native_space_and_terminal_ctrl_c_reach_recorder(self):
        child, terminal = pty.fork()
        if child == 0:
            os.execv(sys.executable, [sys.executable, str(SCRIPT),
                     *self.args("0"), "--", sys.executable, str(self.fake), str(self.root)])
        try:
            self.wait_file("started")
            os.write(terminal, b" ")
            self.wait_file("space")
            os.write(terminal, b"\x03")
            self.wait_file("saved")
        finally:
            try:
                os.kill(child, signal.SIGINT)
            except ProcessLookupError:
                pass
            os.waitpid(child, 0)
            os.close(terminal)

    def test_existing_output_is_never_overwritten(self):
        bag = self.root / "bag"
        bag.mkdir()
        marker = bag / "metadata.yaml"
        marker.write_text("original")
        result = subprocess.run(
            [sys.executable, str(SCRIPT), *self.args(), "--check-only"],
            capture_output=True, text=True, timeout=3)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(marker.read_text(), "original")

    def test_space_floor_is_always_active_when_time_limit_disabled(self):
        self.assertIsNone(guard.stop_reason(101, 100, 100000, 0))
        self.assertIn("reserve", guard.stop_reason(100, 100, 0, 0))


if __name__ == "__main__":
    unittest.main()
