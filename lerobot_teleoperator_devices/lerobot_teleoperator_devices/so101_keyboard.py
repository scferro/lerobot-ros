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

from .config_so101_keyboard import SO101KeyboardConfig
from .keyboard_joint import KeyboardJointTeleop


class SO101Keyboard(KeyboardJointTeleop):
    """
    SO-101 keyboard teleoperation for single arm.

    Provides keyboard control for one SO-101 arm with proper joint naming.

    Keyboard Layout:
    ================
    - Q/A: shoulder_pan (decrease/increase)
    - W/S: shoulder_lift
    - E/D: elbow_flex
    - R/F: wrist_flex
    - T/G: wrist_roll
    - Y/H: gripper (open/close)

    Usage:
        This class inherits all functionality from KeyboardJointTeleop.
        The only difference is that it uses SO101-specific joint names
        (shoulder_pan, shoulder_lift, etc.) instead of generic numbered keys.
    """

    config_class = SO101KeyboardConfig
    name = "so101_keyboard"

    # All implementation inherited from KeyboardJointTeleop
    # The config.arm_action_keys determines the joint names
