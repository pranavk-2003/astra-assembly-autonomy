"""Subprocess bridge to scripts/curobo_plan_server.py.

cuRobo's venv is Python 3.11; ROS 2 Jazzy is 3.12. rclpy's C ext won't load
in cuRobo's venv, so the two can't share an interpreter - real constraint,
not a workaround. Bridge talks JSON-per-line over stdin/stdout to a
long-lived subprocess running cuRobo's own python."""
from __future__ import annotations

import json
import subprocess
import threading
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_VENV_PY = Path.home() / "client_assignment" / "curobo" / ".venv" / "bin" / "python3"
DEFAULT_SERVER = REPO_ROOT / "scripts" / "curobo_plan_server.py"


class CuRoboBridge:
    def __init__(self, robot: str = "ur10e.yml", venv_python: Path = DEFAULT_VENV_PY,
                 server_script: Path = DEFAULT_SERVER, startup_timeout_s: float = 120.0):
        self._proc = subprocess.Popen(
            [str(venv_python), str(server_script), "--robot", robot],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1,
        )
        self._lock = threading.Lock()
        ready = self._recv(timeout_s=startup_timeout_s)
        if ready is None or not ready.get("ok"):
            err = self._proc.stderr.read() if self._proc.stderr else ""
            raise RuntimeError(f"cuRobo server failed to start: {ready} {err[:2000]}")
        self.joint_names: list[str] = ready["joints"]
        self.tool_frames: list[str] = ready.get("tool_frames", [])

    def _recv(self, timeout_s: float) -> dict | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._proc.poll() is not None:
                return None
            line = self._proc.stdout.readline()
            if line:
                try:
                    return json.loads(line)
                except json.JSONDecodeError:
                    continue
        return None

    def plan(self, xyz, quat_wxyz, start_positions: list[float],
             obstacles: list[dict], timeout_s: float = 15.0) -> dict:
        """One plan call. Returns the raw response dict - {"ok": True,
        "trajectory": [[q..],..], "joints": [...], "dt": float} or
        {"ok": False, "reason": str}."""
        request = {
            "cmd": "plan",
            "goal": {"xyz": list(xyz), "quat_wxyz": list(quat_wxyz)},
            "start": list(start_positions),
            "obstacles": obstacles,
        }
        with self._lock:
            self._proc.stdin.write(json.dumps(request) + "\n")
            self._proc.stdin.flush()
            response = self._recv(timeout_s=timeout_s)
        if response is None:
            return {"ok": False, "reason": "cuRobo server timed out or died"}
        return response

    def shutdown(self) -> None:
        try:
            with self._lock:
                self._proc.stdin.write(json.dumps({"cmd": "shutdown"}) + "\n")
                self._proc.stdin.flush()
        except Exception:
            pass
        self._proc.terminate()
