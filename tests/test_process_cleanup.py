import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]


def test_stop_group_kills_term_ignoring_leader(tmp_path):
    child = subprocess.Popen(
        [sys.executable, "-c", (
            "import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); "
            "print('READY', flush=True); time.sleep(30)"
        )],
        start_new_session=True,
        stdout=subprocess.PIPE,
        text=True,
    )
    try:
        assert child.stdout is not None
        assert child.stdout.readline().strip() == "READY"
        started = time.monotonic()
        result = subprocess.run(
            ["bash", "-c", 'source "$1"; stop_group "$2"', "test",
             str(ROOT / "scripts/process_helpers.sh"), str(child.pid)],
            capture_output=True,
            text=True,
            timeout=4,
        )
        elapsed = time.monotonic() - started
        child_status = child.wait(timeout=1)
        report = {
            "helper_exit": result.returncode,
            "term_ignoring_leader_exit": child_status,
            "elapsed_sec": elapsed,
        }
        (tmp_path / "bounded-cleanup.json").write_text(
            json.dumps(report, indent=2) + "\n"
        )
        assert result.returncode == 0
        assert child_status == -signal.SIGKILL
        assert elapsed < 3
    finally:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
