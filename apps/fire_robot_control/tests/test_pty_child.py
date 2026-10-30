import os
import sys
import time
import unittest

from services.pairing import PairingOutputParser
from services.pty_child import PtyChild


class PtyChildTests(unittest.TestCase):
    def setUp(self):
        self.children = []

    def tearDown(self):
        for child in self.children:
            # These are local synthetic test children, never robot processes.
            if child.poll() is None:
                child.process.kill()
                child.process.wait(timeout=2)
            child.close_after_exit()

    def child(self, script):
        child = PtyChild([sys.executable, "-u", "-c", script])
        self.children.append(child)
        return child

    def collect_until(self, child, marker, timeout=3):
        collected = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and marker not in collected:
            collected += child.read()
            time.sleep(.01)
        self.assertIn(marker, collected)
        return collected

    def test_controlling_terminal_and_input_is_not_echoed(self):
        child = self.child(
            "import os,sys; print('CONTROLLING='+str(os.isatty(0) and "
            "os.tcgetpgrp(0)==os.getpgrp()),flush=True); "
            "print('PASSWORD?',flush=True); x=input(); print('ACCEPTED='+str(x=='fixture-secret'),flush=True)")
        before = self.collect_until(child, b"PASSWORD?")
        self.assertIn(b"CONTROLLING=True", before)
        self.assertEqual(child.write(b"fixture-secret\n"), len(b"fixture-secret\n"))
        after = self.collect_until(child, b"ACCEPTED=True")
        self.assertNotIn(b"fixture-secret", before + after)
        child.process.wait(timeout=2)
        self.assertEqual(child.poll(), 0)

    def test_parser_reads_announcements_from_real_local_child(self):
        root = "/home/pi/qa_logs/r5_automatic_20261029_123456"
        child = self.child("import sys,time; "
            "sys.stdout.write('noise\\nLOG_PI=" + root + "\\nPI_PAIR'); sys.stdout.flush(); "
            "time.sleep(.05); print('ING=" + root + "/pairing.json',flush=True)")
        parser = PairingOutputParser()
        found = []
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not found:
            data = child.read(16)
            if data:
                found.extend(parser.feed(data))
            time.sleep(.005)
        self.assertEqual(len(found), 1)
        self.assertEqual(found[0].pairing_path, root + "/pairing.json")
        child.process.wait(timeout=2)

    def test_running_child_cannot_be_closed_and_exit_is_not_stop_proof(self):
        child = self.child("print('ready',flush=True); input()")
        self.collect_until(child, b"ready")
        with self.assertRaises(RuntimeError):
            child.close_after_exit()
        child.write(b"done\n")
        child.process.wait(timeout=2)
        self.assertEqual(child.poll(), 0)
        child.close_after_exit()
        with self.assertRaises(BrokenPipeError):
            child.write(b"x")

    def test_bad_inputs_are_rejected(self):
        for argv in ([], [""], ["bad\0argv"], "echo"):
            with self.subTest(argv=argv), self.assertRaises(ValueError):
                PtyChild(argv)
        child = self.child("input()")
        for size in (0, -1, 8193, True):
            with self.subTest(size=size), self.assertRaises(ValueError):
                child.read(size)
        with self.assertRaises(ValueError):
            child.write(b"x" * 4097)
        child.write(b"done\n")
        child.process.wait(timeout=2)


if __name__ == "__main__":
    unittest.main()
