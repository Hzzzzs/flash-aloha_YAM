"""Shutdown regression checks: fake robot only, no SDK imports or CAN access."""

import ast
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
TREE = ast.parse((ROOT / "scripts/10_gravity_comp_static_eval.py").read_text())
CLOSE_SOURCE = ast.unparse(next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == "_close_robot"))


class ShutdownTests(unittest.TestCase):
    def test_broken_log_pipe_still_closes_robot(self):
        namespace = {"signal": signal}
        exec(CLOSE_SOURCE, namespace)
        closed = []
        robot = type("FakeRobot", (), {"close": lambda self: closed.append(True)})()
        previous = signal.getsignal(signal.SIGINT)
        with patch("builtins.print", side_effect=BrokenPipeError):
            namespace["_close_robot"](robot)
        self.assertEqual(closed, [True])
        self.assertEqual(signal.getsignal(signal.SIGINT), previous)

    def test_foreground_ctrl_c_preserves_cleanup_and_log(self):
        self.run_wrapper(interrupt=True)

    def test_normal_exit_preserves_cleanup_and_log(self):
        self.run_wrapper(interrupt=False)

    def run_wrapper(self, interrupt):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "scripts").mkdir()
            (root / "scripts/yam_env.sh").write_text(
                'PYTHON_BIN="$ROOT_DIR/i2rt/.venv/bin/python"\n')
            wrapper = root / "scripts/10_gravity_comp_static_eval.sh"
            wrapper.write_text((ROOT / "scripts/10_gravity_comp_static_eval.sh").read_text())
            binary_dir = root / "i2rt/.venv/bin"
            binary_dir.mkdir(parents=True)
            fake_python = binary_dir / "python"
            fake_python.write_text(
                f"#!{sys.executable}\nimport signal, time, sys\nfrom pathlib import Path\n"
                + f"if sys.argv[1:2] == ['-c']: print({str(root / 'logs')!r}); sys.exit(0)\n"
                + CLOSE_SOURCE + "\n"
                + f"root = Path({str(root)!r})\n"
                + "class FakeRobot:\n"
                + "    def close(self):\n"
                + "        (root / 'closing').touch()\n"
                + "        time.sleep(0.25)\n"
                + "        (root / 'closed').touch()\n"
                + "        print('FAKE motors disabled', flush=True)\n"
                + "try:\n"
                + "    (root / 'ready').touch()\n"
                + f"    time.sleep({30 if interrupt else 0})\n"
                + "finally:\n    _close_robot(FakeRobot())\n"
            )
            fake_python.chmod(0o755)
            fake_ip = binary_dir / "ip"
            fake_ip.write_text("#!/bin/sh\necho 'FAKE CAN: no hardware access'\n")
            fake_ip.chmod(0o755)
            env = dict(os.environ, PATH=f"{binary_dir}:{os.environ['PATH']}")
            process = subprocess.Popen(
                ["bash", str(wrapper), "can_left", "5", "1"],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, start_new_session=True, env=env,
            )
            try:
                process.stdin.write("LEFT_STATIC_EVAL_READY\n")
                process.stdin.flush()
                if interrupt:
                    self.wait_for(root / "ready", process)
                    os.killpg(process.pid, signal.SIGINT)
                    self.wait_for(root / "closing", process)
                    os.killpg(process.pid, signal.SIGINT)
                output, _ = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 130 if interrupt else 0, output)
                self.assertTrue((root / "closed").exists(), output)
                session = next((root / "logs/gravity-comp-static-eval").iterdir())
                self.assertIn("FAKE motors disabled", (session / "evaluation.log").read_text())
                self.assertTrue((session / "can-after.txt").exists(), output)
                self.assertIn("Session log:", output)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.communicate()

    def wait_for(self, path, process):
        deadline = time.monotonic() + 5
        while not path.exists():
            if process.poll() is not None or time.monotonic() >= deadline:
                self.fail(f"Fake controller did not reach {path.name}")
            time.sleep(0.01)


if __name__ == "__main__":
    unittest.main()
