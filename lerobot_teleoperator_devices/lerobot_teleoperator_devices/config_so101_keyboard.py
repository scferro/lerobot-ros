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


@TeleoperatorConfig.register_subclass("so101_keyboard")
@dataclass
class SO101KeyboardConfig(KeyboardTeleopConfig):
    """
    Configuration for SO-101 keyboard teleoperation.

    Keyboard mappings (single arm):
    - Q/A: shoulder_pan (decrease/increase)
    - W/S: shoulder_lift
    - E/D: elbow_flex
    - R/F: wrist_flex
    - T/G: wrist_roll
    - Y/H: gripper

    Upper key = decrease, Lower key = increase
    """

    # SO-101 specific joint names
    arm_action_keys: list[str] = field(
        default_factory=lambda: [
            "shoulder_pan.pos",
            "shoulder_lift.pos",
            "elbow_flex.pos",
            "wrist_flex.pos",
            "wrist_roll.pos",
        ]
    )

    gripper_action_key: str = "gripper.pos"

    # The amount by which a joint action changes when a key is pressed
    action_increment: float = 0.02
