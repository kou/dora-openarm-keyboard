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

"""Key bindings for keyboard teleoperation.

The left half of the keyboard drives the left arm and the right half drives the
right arm.  Both halves use the same geometric shape, shifted five columns
across, so each pair straddles its home-row anchor identically::

              LEFT ARM (left hand)        RIGHT ARM (right hand)
  +X / -X          W / S                        U / J
  +Y / -Y          A / D                        H / K
  +Z / -Z          R / F                        O / L
  +Pitch / -Pitch  E / C                        I / ,
  +Yaw   / -Yaw    Q / Z                        Y / N
  +Roll  / -Roll   T / B                        P / /
  gripper close    G                            ;
  gripper open     V                            .
"""

LINEAR = "linear"
ANGULAR = "angular"
GRIP = "grip"

RIGHT = "right"
LEFT = "left"

# Axis indices shared by LINEAR (x, y, z) and ANGULAR (roll, pitch, yaw).
X = ROLL = 0
Y = PITCH = 1
Z = YAW = 2

# key -> (arm, kind, axis, sign).  For GRIP, sign +1 closes and -1 opens.
KEYMAP: dict[str, tuple[str, str, int, int]] = {
    # ── left arm ─────────────────────────────────────────────────────────────
    "w": (LEFT, LINEAR, X, +1),
    "s": (LEFT, LINEAR, X, -1),
    "a": (LEFT, LINEAR, Y, +1),
    "d": (LEFT, LINEAR, Y, -1),
    "r": (LEFT, LINEAR, Z, +1),
    "f": (LEFT, LINEAR, Z, -1),
    "e": (LEFT, ANGULAR, PITCH, +1),
    "c": (LEFT, ANGULAR, PITCH, -1),
    "q": (LEFT, ANGULAR, YAW, +1),
    "z": (LEFT, ANGULAR, YAW, -1),
    "t": (LEFT, ANGULAR, ROLL, +1),
    "b": (LEFT, ANGULAR, ROLL, -1),
    "g": (LEFT, GRIP, 0, +1),
    "v": (LEFT, GRIP, 0, -1),
    # ── right arm ────────────────────────────────────────────────────────────
    "u": (RIGHT, LINEAR, X, +1),
    "j": (RIGHT, LINEAR, X, -1),
    "h": (RIGHT, LINEAR, Y, +1),
    "k": (RIGHT, LINEAR, Y, -1),
    "o": (RIGHT, LINEAR, Z, +1),
    "l": (RIGHT, LINEAR, Z, -1),
    "i": (RIGHT, ANGULAR, PITCH, +1),
    ",": (RIGHT, ANGULAR, PITCH, -1),
    "y": (RIGHT, ANGULAR, YAW, +1),
    "n": (RIGHT, ANGULAR, YAW, -1),
    "p": (RIGHT, ANGULAR, ROLL, +1),
    "/": (RIGHT, ANGULAR, ROLL, -1),
    ";": (RIGHT, GRIP, 0, +1),
    ".": (RIGHT, GRIP, 0, -1),
}

# Edge-triggered control keys, handled on key-down rather than while held.
RESET_KEY = "backspace"
SPEED_UP_KEYS = ("+", "=")
SPEED_DOWN_KEYS = ("-", "_")

HELP_TEXT = """\
              LEFT ARM (left hand)   RIGHT ARM (right hand)
  +X / -X          W / S                    U / J
  +Y / -Y          A / D                    H / K
  +Z / -Z          R / F                    O / L
  +Pitch / -Pitch  E / C                    I / ,
  +Yaw   / -Yaw    Q / Z                    Y / N
  +Roll  / -Roll   T / B                    P / /
  gripper close    G                        ;
  gripper open     V                        .

  + / -      speed scale up / down
  Backspace  reset both arms to their home pose\
"""
