#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

from dataclasses import dataclass, field

from lerobot.teleoperators import TeleoperatorConfig
from lerobot.teleoperators.keyboard import KeyboardTeleopConfig


@TeleoperatorConfig.register_subclass("bi_so101_keyboard")
@dataclass
class BiSO101KeyboardConfig(KeyboardTeleopConfig):
    """
    Configuration for bimanual SO-101 keyboard teleoperation.

    Keyboard mappings:
    - Left arm (5 joints):
      - Q/A: left_shoulder_pan
      - W/S: left_shoulder_lift
      - E/D: left_elbow_flex
      - R/F: left_wrist_flex
      - T/G: left_wrist_roll
      - Y/H: left_gripper

    - Right arm (5 joints):
      - U/J: right_shoulder_pan
      - I/K: right_shoulder_lift
      - O/L: right_elbow_flex
      - P/;: right_wrist_flex
      - [/': right_wrist_roll
      - ]/\\: right_gripper

    Upper key = decrease, Lower key = increase
    """

    # Left arm action keys (without prefix)
    left_arm_action_keys: list[str] = field(
        default_factory=lambda: [
            "shoulder_pan.pos",
            "shoulder_lift.pos",
            "elbow_flex.pos",
            "wrist_flex.pos",
            "wrist_roll.pos",
        ]
    )

    # Right arm action keys (without prefix)
    right_arm_action_keys: list[str] = field(
        default_factory=lambda: [
            "shoulder_pan.pos",
            "shoulder_lift.pos",
            "elbow_flex.pos",
            "wrist_flex.pos",
            "wrist_roll.pos",
        ]
    )

    # Gripper action keys (without prefix)
    left_gripper_action_key: str = "gripper.pos"
    right_gripper_action_key: str = "gripper.pos"

    # The amount by which a joint action changes when a key is pressed
    action_increment: float = 0.02
