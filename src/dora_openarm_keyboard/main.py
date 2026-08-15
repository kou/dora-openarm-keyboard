# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Keyboard teleoperation node.

Publishes end-effector pose targets driven by the keyboard, using the same
output contract as dora-openarm-vr so either can feed dora-openarm-ik:

  pose_right / pose_left : [{"pose": float32[8]}]
      [px, py, pz, qw, qx, qy, qz, gripper_angle] in the scene's ``arm_origin``
      frame.
  status : string[1]

Keys are captured globally with pynput, which needs Accessibility permission on
macOS (System Settings -> Privacy & Security -> Accessibility).  Because capture
is global, the motion keys drive the robot from whatever window has focus; they
only ever move the target while they are held.

Key events are queued by the listener thread and drained on the dora thread, so
the teleoperation state is only ever mutated from one thread.
"""

import argparse
import queue
import sys
import time

import dora
import numpy as np
import pyarrow as pa
from pynput import keyboard
from scipy.spatial.transform import Rotation

from .keymap import (
    HELP_TEXT,
    LEFT,
    RESET_KEY,
    RIGHT,
    SPEED_DOWN_KEYS,
    SPEED_UP_KEYS,
)
from .teleop import (
    DEFAULT_ANGULAR_SPEED,
    DEFAULT_GRIP_SPEED,
    DEFAULT_HOME_LEFT,
    DEFAULT_HOME_RIGHT,
    DEFAULT_HOME_RPY,
    DEFAULT_LINEAR_SPEED,
    DEFAULT_POS_MAX,
    DEFAULT_POS_MIN,
    KeyState,
    TeleopState,
)

_POSE_STRUCT_TYPE = pa.struct({"pose": pa.list_(pa.float32())})

_SPEED_STEP = 1.25

# A stalled dataflow must not teleport the target on the next tick.
_MAX_DT = 0.1

# How long to wait before pointing out that the listener looks dead.
_SILENT_LISTENER_SECONDS = 10.0

_SILENT_LISTENER_PREFIX = "pynput is not delivering keystrokes to this process."

_SPECIAL_KEYS = {
    keyboard.Key.backspace: RESET_KEY,
}

_CONTROL_KEYS = frozenset((RESET_KEY, *SPEED_UP_KEYS, *SPEED_DOWN_KEYS))


_ACCESSIBILITY_HINT = (
    "Grant Accessibility permission to the program that launched dora — your "
    "terminal, or your IDE if you started it from there — under System "
    "Settings -> Privacy & Security -> Accessibility, then restart the "
    "dataflow. The permission belongs to that program, not to python itself."
)


def accessibility_state() -> bool | None:
    """Report whether macOS trusts this process to observe input events.

    pynput's macOS backend starts happily without Accessibility permission and
    then never delivers an event, so an explicit check is the only way to tell
    the operator why nothing moves.  Returns None where the question does not
    apply or cannot be answered.
    """
    if sys.platform != "darwin":
        return None
    try:
        from ApplicationServices import AXIsProcessTrusted
    except ImportError:
        return None
    return bool(AXIsProcessTrusted())


def build_pose_output(pose: np.ndarray) -> pa.Array:
    """Wrap a pose array as a length-1 StructArray: [{"pose": [...]}]."""
    return pa.array([{"pose": pose}], type=_POSE_STRUCT_TYPE)


def normalize_key(key) -> str | None:
    """Map a pynput key to its keymap name, or None if it is not usable."""
    special = _SPECIAL_KEYS.get(key)
    if special is not None:
        return special
    char = getattr(key, "char", None)
    if not char:
        return None
    return char.lower()


class KeyboardTeleop:
    """Drains queued key events and turns held keys into pose targets."""

    def __init__(self, state: TeleopState) -> None:
        """Wrap a teleoperation state with the key event plumbing."""
        self.state = state
        self.keys = KeyState()
        self.events: queue.SimpleQueue = queue.SimpleQueue()
        self._control_down: set[str] = set()
        self._status: str | None = None
        self.events_seen = 0

    # ── listener thread ──────────────────────────────────────────────────────

    def on_press(self, key) -> None:
        """Queue a key-down event. Called from the listener thread."""
        name = normalize_key(key)
        if name is not None:
            self.events.put(("press", name))

    def on_release(self, key) -> None:
        """Queue a key-up event. Called from the listener thread."""
        name = normalize_key(key)
        if name is not None:
            self.events.put(("release", name))

    # ── dora thread ──────────────────────────────────────────────────────────

    def drain(self) -> None:
        """Apply every queued key event in the order it arrived."""
        while True:
            try:
                action, name = self.events.get_nowait()
            except queue.Empty:
                return
            self.events_seen += 1
            if self.events_seen == 1:
                print(
                    f"[teleop] keyboard listener is delivering events "
                    f"(first: {name!r})",
                    flush=True,
                )
            if name not in _CONTROL_KEYS:
                if action == "press":
                    self.keys.press(name)
                else:
                    self.keys.release(name)
                continue

            if action == "release":
                self._control_down.discard(name)
            elif name not in self._control_down:
                # Edge-triggered: ignore the operating system's key repeat.
                self._control_down.add(name)
                self._handle_control(name)

    def _handle_control(self, name: str) -> None:
        if name == RESET_KEY:
            self.state.reset()
            self._note("reset to home")
        elif name in SPEED_UP_KEYS:
            self._note(f"speed scale {self.state.scale_speed(_SPEED_STEP):.2f}")
        elif name in SPEED_DOWN_KEYS:
            self._note(f"speed scale {self.state.scale_speed(1 / _SPEED_STEP):.2f}")

    def _note(self, status: str) -> None:
        self._status = status
        print(f"[teleop] {status} | {self.state.describe()}", flush=True)

    def take_status(self) -> str | None:
        """Return and clear the status set since the last call."""
        status, self._status = self._status, None
        return status

    def step(self, dt: float) -> None:
        """Apply queued key events, then integrate one timestep."""
        self.drain()
        self.state.step(dt, self.keys.held)


def _run(args: argparse.Namespace) -> None:
    state = TeleopState(
        home_right=np.array(args.home_right, dtype=np.float64),
        home_left=np.array(args.home_left, dtype=np.float64),
        home_rotation=Rotation.from_euler("xyz", args.home_rpy, degrees=True),
        linear_speed=args.linear_speed,
        angular_speed=args.angular_speed,
        grip_speed=args.grip_speed,
        pos_min=np.array(args.pos_min, dtype=np.float64),
        pos_max=np.array(args.pos_max, dtype=np.float64),
    )
    teleop = KeyboardTeleop(state)

    print(HELP_TEXT, flush=True)
    print(
        "[teleop] keys are captured globally — they drive the robot from "
        "whatever window has focus.",
        flush=True,
    )

    trusted = accessibility_state()
    if trusted is not None:
        # Only ever context: this reports False on setups whose events arrive
        # anyway, so the silent-listener check below is the real verdict.
        print(f"[teleop] macOS accessibility trusted: {trusted}", flush=True)

    listener = keyboard.Listener(on_press=teleop.on_press, on_release=teleop.on_release)
    try:
        listener.start()
        listener.wait()
    except Exception as error:  # noqa: BLE001 - the node stays useful either way
        print(f"[teleop] keyboard listener did not start: {error}", flush=True)
    else:
        print(f"[teleop] keyboard listener running: {listener.running}", flush=True)

    node = dora.Node()
    node.send_output("status", pa.array(["ready"]))

    last = time.perf_counter()
    started = last
    warned_silent = False
    try:
        for event in node:
            if event["type"] != "INPUT" or event["id"] != "tick":
                continue

            now = time.perf_counter()
            dt = min(now - last, _MAX_DT)
            last = now

            if (
                not warned_silent
                and teleop.events_seen == 0
                and now - started > _SILENT_LISTENER_SECONDS
            ):
                warned_silent = True
                print(
                    f"[teleop] WARNING: no key event in "
                    f"{_SILENT_LISTENER_SECONDS:.0f}s. "
                    f"{_SILENT_LISTENER_PREFIX} {_ACCESSIBILITY_HINT}",
                    flush=True,
                )

            teleop.step(dt)

            metadata = {"timestamp": time.time_ns()}
            node.send_output(
                "pose_right", build_pose_output(state.pose(RIGHT)), metadata
            )
            node.send_output("pose_left", build_pose_output(state.pose(LEFT)), metadata)

            status = teleop.take_status()
            if status is not None:
                node.send_output("status", pa.array([status]), metadata)
    finally:
        listener.stop()


def build_parser() -> argparse.ArgumentParser:
    """Build the command line parser."""
    parser = argparse.ArgumentParser(
        description="Keyboard teleoperation for OpenArm (dora node)"
    )
    parser.add_argument(
        "--linear-speed",
        type=float,
        default=DEFAULT_LINEAR_SPEED,
        help="translation speed in m/s (default: %(default)s)",
    )
    parser.add_argument(
        "--angular-speed",
        type=float,
        default=DEFAULT_ANGULAR_SPEED,
        help="rotation speed in rad/s (default: %(default)s)",
    )
    parser.add_argument(
        "--grip-speed",
        type=float,
        default=DEFAULT_GRIP_SPEED,
        help="gripper speed in fraction/s (default: %(default)s)",
    )
    parser.add_argument(
        "--home-right",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=DEFAULT_HOME_RIGHT.tolist(),
        help="right arm home position in the arm_origin frame",
    )
    parser.add_argument(
        "--home-left",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=DEFAULT_HOME_LEFT.tolist(),
        help="left arm home position in the arm_origin frame",
    )
    parser.add_argument(
        "--home-rpy",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=list(DEFAULT_HOME_RPY),
        help="home orientation in degrees, shared by both arms",
    )
    parser.add_argument(
        "--pos-min",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=DEFAULT_POS_MIN.tolist(),
        help="lower workspace bound",
    )
    parser.add_argument(
        "--pos-max",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=DEFAULT_POS_MAX.tolist(),
        help="upper workspace bound",
    )
    return parser


def main() -> None:
    """Run the keyboard teleoperation node."""
    _run(build_parser().parse_args())


if __name__ == "__main__":
    main()
