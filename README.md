# dora-openarm-keyboard

A [dora-rs](https://dora-rs.ai/) node that teleoperates OpenArm from
the keyboard in a Web browser.

The node serves a web page (default `http://127.0.0.1:8080/`) that does
both halves of teleoperation in one browser tab:

- **Keys in**: the page captures the key bindings below and sends them
  over a WebRTC data channel. They only work while the tab has focus,
  and losing focus (or closing the tab, or losing the network) releases
  every held key — an unwatched browser can never keep the robot moving.
- **Video out**: JPEG frames arriving on the node's optional `image`
  input are streamed to the page as WebRTC video. Wire it to a
  `camera_*` output of `dora-openarm-mujoco --render` to watch the
  simulation from the browser.

It publishes end-effector pose targets with the same output contract
as
[`dora-openarm-webxr`](https://github.com/enactic/dora-openarm-webxr),
so it can be dropped into a dataflow wherever the WebXR node would
normally sit and feed
[`dora-openarm-ik`](https://github.com/enactic/dora-openarm-kinematics)
unchanged.

## Key bindings

The left half of the keyboard drives the left arm and the right half drives the
right arm. Both halves use the same shape, shifted five columns across, so each
pair straddles its home-row anchor identically — `W`/`S` around `A` on the left
is `U`/`J` around `H` on the right.

| | Left arm (left hand) | Right arm (right hand) |
|---|---|---|
| **+X / -X** | <kbd>W</kbd> / <kbd>S</kbd> | <kbd>U</kbd> / <kbd>J</kbd> |
| **+Y / -Y** | <kbd>A</kbd> / <kbd>D</kbd> | <kbd>H</kbd> / <kbd>K</kbd> |
| **+Z / -Z** | <kbd>R</kbd> / <kbd>F</kbd> | <kbd>O</kbd> / <kbd>L</kbd> |
| **+Pitch / -Pitch** | <kbd>E</kbd> / <kbd>C</kbd> | <kbd>I</kbd> / <kbd>,</kbd> |
| **+Yaw / -Yaw** | <kbd>Q</kbd> / <kbd>Z</kbd> | <kbd>Y</kbd> / <kbd>N</kbd> |
| **+Roll / -Roll** | <kbd>T</kbd> / <kbd>B</kbd> | <kbd>P</kbd> / <kbd>/</kbd> |
| **Gripper close** | <kbd>G</kbd> | <kbd>;</kbd> |
| **Gripper open** | <kbd>V</kbd> | <kbd>.</kbd> |

| Key | Action |
|---|---|
| <kbd>+</kbd> / <kbd>-</kbd> | Speed scale up / down (×1.25 per press, clamped to 0.1–10) |
| <kbd>Backspace</kbd> | Reset both arms to their home pose |

Motion keys are **hold to move**: the target advances while the key is down and
stops the moment it is released. Rotation is integrated in the **tool frame**,
so roll, pitch and yaw stay relative to the gripper rather than the world.

There is no arm/disarm and no stop key: releasing the keys *is* the stop, and a
node nobody is touching publishes the pose it already holds. Keys only reach
the robot while the browser page has focus, and losing focus releases
everything held, so switching windows is itself safe.

By default the page is only reachable from the node's own machine. To operate
from another machine, pass `--host 0.0.0.0` and open `http://<node-host>:8080/`
(browsers allow WebRTC on plain HTTP; camera/mic-free pages like this one need
no HTTPS).

## When the keys do nothing

Check the page: is it open, does its header say *connected*, and does the tab
actually have focus (click the page once)?

## Interface

| | |
|---|---|
| **Inputs** | `tick` — keep-alive only, any rate works (the node integrates and publishes at its own 500 Hz pace); `image` (optional) — JPEG frame to stream to the browser, e.g. a `camera_*` output of `dora-openarm-mujoco --render` |
| **Outputs** | `pose_right`, `pose_left` `[{"pose": float32[8]}]` — `[px, py, pz, qw, qx, qy, qz, gripper_angle]` in the scene's `arm_origin` frame; `status` `string[1]` |

```
--linear-speed   translation speed, m/s        (default: 0.05)
--angular-speed  rotation speed, rad/s         (default: 0.5)
--grip-speed     gripper speed, fraction/s     (default: 2.0)
--home-right     right arm home position X Y Z (default: 0.216 -0.1535 -0.22)
--home-left      left arm home position X Y Z  (default: 0.216 0.1535 -0.22)
--home-rpy       home orientation in degrees   (default: 0 -90 0)
--pos-min        lower workspace bound X Y Z   (default: -0.8 -0.8 -0.8)
--pos-max        upper workspace bound X Y Z   (default: 0.8 0.8 0.8)
--host           web server bind address       (default: 127.0.0.1)
--port           web server port               (default: 8080)
```

The `--host` and `--port` defaults can also be overridden with the `HOST`
and `PORT` environment variables; explicit `--host`/`--port` still win.

The home pose defaults are the end-effector poses of the scene's `home`
keyframe, expressed in its `arm_origin` frame. The IK and MuJoCo nodes start
from that same keyframe, so publishing anything else would make IK drag both
arms across the workspace on the first tick. After changing scene or keyframe,
we need to follow the changes.

## Quick start

[`example/dataflow-mujoco.yaml`](example/dataflow-mujoco.yaml) drives both arms
in MuJoCo through IK, with no VR headset and no real OpenArm, and streams
MuJoCo's ceiling camera back to the browser:

```bash
uv run dora build example/dataflow-mujoco.yaml --uv
uv run dora run example/dataflow-mujoco.yaml --uv
```

Then open <http://127.0.0.1:8080/> and click the page. The arms hold their
startup pose until you press a key.

To record what you teleoperate, use `dataflow-keyboard-mujoco.yaml` in
[`dora-openarm-data-collection`](https://github.com/enactic/dora-openarm-data-collection)
— the same graph with the collection UI and dataset recorder attached.

## Development

```bash
uv sync
uv run pytest tests
```

The pose integrator in `teleop.py` imports neither `dora` nor the WebRTC
stack, so the whole state machine is tested without a dataflow or a browser;
`tests/test_web.py` exercises the WebRTC server end to end in-process.

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
