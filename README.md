# dora-openarm-keyboard

A [dora-rs](https://dora-rs.ai/) node that teleoperates OpenArm from
the keyboard.

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
node nobody is touching publishes the pose it already holds. Keys are captured
globally though, so the motion keys reach the robot from whatever window has
focus — quit the dataflow before typing elsewhere.

## When the keys do nothing

The node logs enough at startup to tell you where it stopped:

```
[teleop] keys are captured globally — they drive the robot from whatever window has focus.
[teleop] macOS accessibility trusted: False
[teleop] keyboard listener running: True
[teleop] keyboard listener is delivering events (first: 'w')
```

The last line appears on your first keystroke and is the one that matters — it
proves keys are getting through. If it never appears, the node says so after ten
seconds.

**Is the `delivering events` line there?** If not, macOS is withholding global
capture. Grant Accessibility to *the program that launched dora* — your
terminal, or your IDE if you started it from there — under **System Settings →
Privacy & Security → Accessibility**, then restart the dataflow. The permission
belongs to that program, not to `python`.

The `accessibility trusted` line is context, not a verdict: `AXIsProcessTrusted`
reports `False` on setups whose events arrive perfectly well, so trust the
`delivering events` line over it.

## Interface

| | |
|---|---|
| **Inputs** | `tick` — one integration step per event |
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
```

The home pose defaults are the end-effector poses of the scene's `home`
keyframe, expressed in its `arm_origin` frame. The IK and MuJoCo nodes start
from that same keyframe, so publishing anything else would make IK drag both
arms across the workspace on the first tick. After changing scene or keyframe,
regenerate them:

```bash
dev/home_pose.py --xml path/to/scene.xml --keyframe home
```

## Quick start

[`example/dataflow-mujoco.yaml`](example/dataflow-mujoco.yaml) drives both arms
in the MuJoCo viewer through IK, with no VR headset and no real OpenArm:

```bash
uv run dora build example/dataflow-mujoco.yaml --uv
uv run dora run example/dataflow-mujoco.yaml --uv
```

The arms hold their startup pose until you press a key.

On macOS the MuJoCo viewer only opens under `mjpython`, so point the `viewer`
node's `path` at a wrapper that execs it instead of at `dora-openarm-mujoco`.

To record what you teleoperate, use `dataflow-keyboard-mujoco.yaml` in
[`dora-openarm-data-collection`](https://github.com/enactic/dora-openarm-data-collection)
— the same graph with the collection UI and dataset recorder attached.

## Development

```bash
uv sync
uv run pytest tests
```

The pose integrator in `teleop.py` imports neither `dora` nor `pynput`, so the
whole state machine is tested without a dataflow or a keyboard.

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
