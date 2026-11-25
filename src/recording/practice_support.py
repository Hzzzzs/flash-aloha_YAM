"""Lightweight support for the camera-free practice/replay entry point.

This module deliberately imports neither dataset storage nor training.  It
still samples through the mandatory ``standard_capture`` boundary.
"""
from __future__ import annotations

from dataclasses import dataclass
import os
import queue
import select
import sys
import termios
import threading
import time
import tty

import numpy as np

from contracts import standard_capture


@dataclass
class RunContext:
    task_name: str
    dataset_dir: str
    max_timesteps: int
    camera_names: list[str]
    robot_type: str
    single_arm: bool
    double_arm: bool
    can_interface_left: str | None = None
    can_interface_right: str | None = None
    arx_single_arm_side: str | None = None
    fps: float = 100.0
    use_start_position: bool = False
    start_position_path: str | None = None
    start_position_move_duration: float = 3.0
    robot_contract: dict | None = None
    hardware_config: dict | None = None


@dataclass
class EpisodeSample:
    images: dict
    qpos: np.ndarray
    qvel: np.ndarray
    timestamp: float
    robot_timestamp: float
    robot_read_started_at: float
    camera_observation: dict
    capture_contract_version: int


class EpisodeCapture:
    """100 Hz camera-free capture using the repository's standard boundary."""

    def __init__(self, backend, camera_names, *, fps, clock=None, **unused):
        if camera_names:
            raise ValueError("PracticeCapture is camera-free")
        if not np.isfinite(fps) or fps <= 0:
            raise ValueError("Capture fps must be positive and finite")
        self.backend, self.fps = backend, float(fps)
        self.clock = clock or time.monotonic
        self.started = self.next_tick = self.clock()

    def wait_and_sample(self):
        while self.clock() < self.next_tick:
            time.sleep(max(0.0, self.next_tick - self.clock()))
        loop_start = self.clock()
        self.backend.passive_step()
        observation = standard_capture.capture_observation(
            self.backend.read_state, [], clock=self.clock, include_images=False)
        self.next_tick = max(self.next_tick + 1 / self.fps, loop_start)
        return EpisodeSample(
            images=observation.images,
            qpos=observation.qpos,
            qvel=observation.qvel,
            timestamp=observation.robot_timestamp - self.started,
            robot_timestamp=observation.robot_timestamp,
            robot_read_started_at=observation.robot_read_started_at,
            camera_observation=observation.camera_observation,
            capture_contract_version=standard_capture.CAPTURE_CONTRACT_VERSION)


class TerminalKeyWatcher:
    """One terminal reader shared by the practice UI and safe-exit home key."""

    def __init__(self):
        self.keys: queue.Queue[str] = queue.Queue()
        self.stop = threading.Event()
        self.fd = sys.stdin.fileno()
        self.saved = None
        self.thread = None

    def __enter__(self):
        if os.isatty(self.fd):
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        self.thread = threading.Thread(target=self._read, name="practice-key-watcher", daemon=True)
        self.thread.start()
        return self

    def _read(self):
        while not self.stop.is_set():
            ready, _, _ = select.select([self.fd], [], [], .1)
            if not ready:
                continue
            data = os.read(self.fd, 16)
            if not data:
                return
            text = data.decode(errors="ignore").lower()
            # Keep ANSI arrows as one key; ordinary bytes are independent keys.
            if text.startswith("\x1b[") and len(text) >= 3:
                self.keys.put(text[:3])
                text = text[3:]
            for key in text:
                self.keys.put(key)

    def poll(self):
        try:
            return self.keys.get_nowait()
        except queue.Empty:
            return None

    def home_requested(self):
        key = self.poll()
        return key is not None and key.lower() == "h"

    def __exit__(self, exc_type, exc, traceback):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=.2)
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
