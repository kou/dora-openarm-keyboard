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

Keys come from a browser page the node itself serves: they travel over a WebRTC
data channel and only work while the page has focus, so an unwatched browser
can never keep the robot moving.  The same page shows the MuJoCo camera stream
fed into the optional ``image`` input (JPEG frames, e.g. a ``camera_*`` output
of dora-openarm-mujoco ``--render``) as WebRTC video.

The dora node and the WebRTC stack share one asyncio loop: dora events are
polled with ``node.is_empty()`` between short sleeps, so the WebRTC tasks stay
live without any extra thread.  Pose targets are integrated and published by a
self-paced task; the dataflow's ``tick`` input is only a keep-alive and its
rate does not matter.  Key events are queued by the WebRTC handlers and
drained once per integration step, so the teleoperation state only changes
there.
"""

import argparse
import asyncio
import os
import queue
import time

import dora
import numpy as np
import pyarrow as pa
from scipy.spatial.transform import Rotation

from .keymap import (
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
from .web import WebTeleopServer

_POSE_STRUCT_TYPE = pa.struct({"pose": pa.list_(pa.float32())})

_SPEED_STEP = 1.25

# Integration step period; the node paces itself instead of following a tick.
_STEP_SECONDS = 0.002

# A stalled loop must not teleport the target on the next step.
_MAX_DT = 0.1

_CONTROL_KEYS = frozenset((RESET_KEY, *SPEED_UP_KEYS, *SPEED_DOWN_KEYS))


def build_pose_output(pose: np.ndarray) -> pa.Array:
    """Wrap a pose array as a length-1 StructArray: [{"pose": [...]}]."""
    return pa.array([{"pose": pose}], type=_POSE_STRUCT_TYPE)


class KeyboardTeleop:
    """Drains queued key events and turns held keys into pose targets."""

    def __init__(self, state: TeleopState) -> None:
        """Wrap a teleoperation state with the key event plumbing."""
        self.state = state
        self.keys = KeyState()
        self.events: queue.SimpleQueue = queue.SimpleQueue()
        self._control_down: set[str] = set()
        self._status: str | None = None
        # Cleared by the dora loop to let the integrator task finish cleanly.
        self.running = True

    # WebRTC handlers

    def enqueue(self, action: str, name: str) -> None:
        """Queue a normalized key event ("press" or "release")."""
        self.events.put((action, name))

    # integrator task

    def drain(self) -> None:
        """Apply every queued key event in the order it arrived."""
        while True:
            try:
                action, name = self.events.get_nowait()
            except queue.Empty:
                return
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
        print(f"{status} | {self.state.describe()}", flush=True)

    def take_status(self) -> str | None:
        """Return and clear the status set since the last call."""
        status, self._status = self._status, None
        return status

    def step(self, dt: float) -> None:
        """Apply queued key events, then integrate one timestep."""
        self.drain()
        self.state.step(dt, self.keys.held)


def _extract_jpeg(value: pa.Array) -> bytes:
    """Return the raw JPEG payload of an image input (a uint8 Arrow array)."""
    return value.to_numpy(zero_copy_only=False).astype(np.uint8).tobytes()


def _run(args: argparse.Namespace) -> None:
    asyncio.run(_run_async(args))


async def _run_async(args: argparse.Namespace) -> None:
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

    server = WebTeleopServer(on_key=teleop.enqueue, host=args.host, port=args.port)
    if args.offer is not None:
        # WebRTC-only mode: no HTTP server. The offer was handed in at startup
        # and the answer goes back over a TCP socket the caller is listening on;
        # this node then runs that single peer for its whole life. If nobody is
        # listening for the answer, or the browser never connects, exit cleanly
        # instead of dumping a traceback or holding a dead connection.
        if args.answer_port is None:
            raise SystemExit("--answer-port is required when --offer is given")
        try:
            await server.negotiate_oneshot(
                args.offer,
                args.answer_host,
                args.answer_port,
                args.connect_timeout,
            )
        except (OSError, RuntimeError) as error:
            await server.stop()
            raise SystemExit(f"WebRTC-only mode failed: {error}") from None
    else:
        await server.start()

    node = dora.Node()
    node.send_output("status", pa.array(["ready"]))

    integrator = asyncio.create_task(_integrate(node, teleop, state))
    try:
        while True:
            # Poll instead of blocking on the dora iterator: while no event is
            # waiting, the sleep hands the loop to the other tasks (WebRTC and
            # the integrator), which would all stall behind a blocking next().
            if node.is_empty():
                await asyncio.sleep(0.001)
                continue
            event = node.next()
            if event is None or event["type"] == "STOP":
                break
            if event["type"] != "INPUT":
                continue
            if event["id"] == "image":
                server.push_jpeg(_extract_jpeg(event["value"]))
    finally:
        teleop.running = False
        await integrator
        await server.stop()


async def _integrate(
    node: dora.Node,
    teleop: KeyboardTeleop,
    state: TeleopState,
) -> None:
    """Advance the target pose and publish it at the node's own pace."""
    last = time.perf_counter()
    while teleop.running:
        await asyncio.sleep(_STEP_SECONDS)
        now = time.perf_counter()
        dt = min(now - last, _MAX_DT)
        last = now

        teleop.step(dt)

        metadata = {"timestamp": time.time_ns()}
        node.send_output("pose_right", build_pose_output(state.pose(RIGHT)), metadata)
        node.send_output("pose_left", build_pose_output(state.pose(LEFT)), metadata)

        status = teleop.take_status()
        if status is not None:
            node.send_output("status", pa.array([status]), metadata)


def _default_answer_port() -> int | None:
    """Return the default answer port, ANSWER_PORT if set, else None."""
    raw = os.environ.get("ANSWER_PORT")
    if raw is None:
        return None
    try:
        return int(raw)
    except ValueError:
        raise SystemExit(f"ANSWER_PORT must be an integer, got {raw!r}") from None


def _default_connect_timeout() -> float:
    """Return the WebRTC-only connect timeout, CONNECT_TIMEOUT if set."""
    raw = os.environ.get("CONNECT_TIMEOUT", "60")
    try:
        return float(raw)
    except ValueError:
        raise SystemExit(f"CONNECT_TIMEOUT must be a number, got {raw!r}") from None


def _default_port() -> int:
    """Return the default web server port, PORT if set."""
    raw = os.environ.get("PORT", "8080")
    try:
        return int(raw)
    except ValueError:
        raise SystemExit(f"PORT must be an integer, got {raw!r}") from None


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
    parser.add_argument(
        "--host",
        default=os.environ.get("HOST", "127.0.0.1"),
        help="address the web server listens on; use 0.0.0.0 to allow "
        "browsers on other machines; the default can also be set via the "
        "HOST environment variable (default: %(default)s)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=_default_port(),
        help="port the web server listens on; the default can also be set "
        "via the PORT environment variable (default: %(default)s)",
    )
    parser.add_argument(
        "--offer",
        default=os.environ.get("OFFER"),
        help="run in WebRTC-only mode (no HTTP server): the browser's SDP "
        "offer (the bare SDP; its type is always offer), handed in at startup. "
        "The answer SDP is written to --answer-host/--answer-port and this node "
        "then runs that single peer for its whole life; --host/--port are "
        "ignored. Another service hosts the page and brokers signaling. Can "
        "also be set via the OFFER environment variable.",
    )
    parser.add_argument(
        "--answer-host",
        default=os.environ.get("ANSWER_HOST", "127.0.0.1"),
        help="in WebRTC-only mode, the host to connect to and write the answer "
        "SDP to; the default can also be set via the ANSWER_HOST environment "
        "variable (default: %(default)s)",
    )
    parser.add_argument(
        "--answer-port",
        type=int,
        default=_default_answer_port(),
        help="in WebRTC-only mode, the port to connect to and write the answer "
        "SDP to; required when --offer is given, and can also be set via the "
        "ANSWER_PORT environment variable",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=_default_connect_timeout(),
        help="in WebRTC-only mode, seconds to wait for the browser to connect "
        "after the answer is sent before giving up and exiting; the default can "
        "also be set via the CONNECT_TIMEOUT environment variable "
        "(default: %(default)s)",
    )
    return parser


def main() -> None:
    """Run the keyboard teleoperation node."""
    _run(build_parser().parse_args())


if __name__ == "__main__":
    main()
