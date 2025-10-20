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

from queue import Queue
from typing import Any

from lerobot.teleoperators.keyboard import KeyboardTeleop
from lerobot.utils.errors import DeviceNotConnectedError

from .config_bi_so101_keyboard import BiSO101KeyboardConfig


class BiSO101Keyboard(KeyboardTeleop):
    """
    Bimanual SO-101 keyboard teleoperation.

    Provides keyboard control for two SO-101 arms simultaneously.

    Keyboard Layout:
    ================
    LEFT ARM:
    - Q/A: shoulder_pan (decrease/increase)
    - W/S: shoulder_lift
    - E/D: elbow_flex
    - R/F: wrist_flex
    - T/G: wrist_roll
    - Y/H: gripper (open/close)

    RIGHT ARM:
    - U/J: shoulder_pan
    - I/K: shoulder_lift
    - O/L: elbow_flex
    - P/;: wrist_flex
    - [/': wrist_roll
    - ]/\\: gripper
    """

    config_class = BiSO101KeyboardConfig
    name = "bi_so101_keyboard"

    def __init__(self, config: BiSO101KeyboardConfig):
        super().__init__(config)
        self.config = config
        self.misc_keys_queue: Queue[Any] = Queue()

        # Initialize action state for all joints (with prefixes)
        self.curr_joint_actions = {}
        for key in self.config.left_arm_action_keys:
            self.curr_joint_actions[f"left_{key}"] = 0.0
        self.curr_joint_actions[f"left_{self.config.left_gripper_action_key}"] = 0.0

        for key in self.config.right_arm_action_keys:
            self.curr_joint_actions[f"right_{key}"] = 0.0
        self.curr_joint_actions[f"right_{self.config.right_gripper_action_key}"] = 0.0

    @property
    def action_features(self) -> dict:
        """Define action features for both arms."""
        n_joints = (
            len(self.config.left_arm_action_keys)
            + 1  # left gripper
            + len(self.config.right_arm_action_keys)
            + 1  # right gripper
        )

        action_names = {}
        # Left arm
        for key in self.config.left_arm_action_keys:
            action_names[f"left_{key}"] = float
        action_names[f"left_{self.config.left_gripper_action_key}"] = float

        # Right arm
        for key in self.config.right_arm_action_keys:
            action_names[f"right_{key}"] = float
        action_names[f"right_{self.config.right_gripper_action_key}"] = float

        return {
            "dtype": "float32",
            "shape": (n_joints,),
            "names": action_names,
        }

    def _on_press(self, key):
        if hasattr(key, "char"):
            key = key.char
        self.event_queue.put((key, True))

    def _on_release(self, key):
        if hasattr(key, "char"):
            key = key.char
        self.event_queue.put((key, False))

    def get_action(self) -> dict[str, Any]:
        """
        Returns the current joint actions based on pressed keys.

        The action dict contains keys with left_/right_ prefixes matching the
        BiSO101ROS action space.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "BiSO101Keyboard is not connected. You need to run `connect()` before `get_action()`."
            )

        self._drain_pressed_keys()

        # Key mappings: key -> (arm, joint_index, direction)
        # arm: "left" or "right"
        # direction: -1 for decrease, +1 for increase
        key_mappings = {
            # Left arm (5 joints)
            "q": ("left", 0, -1),
            "a": ("left", 0, 1),
            "w": ("left", 1, -1),
            "s": ("left", 1, 1),
            "e": ("left", 2, -1),
            "d": ("left", 2, 1),
            "r": ("left", 3, -1),
            "f": ("left", 3, 1),
            "t": ("left", 4, -1),
            "g": ("left", 4, 1),
            # Right arm (5 joints)
            "u": ("right", 0, -1),
            "j": ("right", 0, 1),
            "i": ("right", 1, -1),
            "k": ("right", 1, 1),
            "o": ("right", 2, -1),
            "l": ("right", 2, 1),
            "p": ("right", 3, -1),
            ";": ("right", 3, 1),
            "[": ("right", 4, -1),
            "'": ("right", 4, 1),
        }

        # Gripper keys: key -> (arm, direction)
        gripper_keys = {
            # Left gripper
            "y": ("left", -1),
            "h": ("left", 1),
            # Right gripper
            "]": ("right", -1),
            "\\": ("right", 1),
        }

        # Process pressed keys and update actions
        for key, val in self.current_pressed.items():
            # Handle arm joint keys
            if key in key_mappings:
                arm, joint_index, direction = key_mappings[key]

                if arm == "left" and joint_index < len(self.config.left_arm_action_keys):
                    joint_key = f"left_{self.config.left_arm_action_keys[joint_index]}"
                    self.curr_joint_actions[joint_key] += direction * self.config.action_increment
                elif arm == "right" and joint_index < len(self.config.right_arm_action_keys):
                    joint_key = f"right_{self.config.right_arm_action_keys[joint_index]}"
                    self.curr_joint_actions[joint_key] += direction * self.config.action_increment

            # Handle gripper keys
            elif key in gripper_keys:
                arm, direction = gripper_keys[key]

                if arm == "left":
                    gripper_key = f"left_{self.config.left_gripper_action_key}"
                    self.curr_joint_actions[gripper_key] += direction * self.config.action_increment
                    # Normalize gripper to 0.0 - 1.0
                    self.curr_joint_actions[gripper_key] = max(
                        0.0, min(1.0, self.curr_joint_actions[gripper_key])
                    )
                else:  # right
                    gripper_key = f"right_{self.config.right_gripper_action_key}"
                    self.curr_joint_actions[gripper_key] += direction * self.config.action_increment
                    # Normalize gripper to 0.0 - 1.0
                    self.curr_joint_actions[gripper_key] = max(
                        0.0, min(1.0, self.curr_joint_actions[gripper_key])
                    )

            elif val:
                # If the key is pressed but doesn't belong to any action, add it to misc_keys_queue
                # this is useful for retrieving other events like interventions for RL, episode success, etc.
                self.misc_keys_queue.put(key)

        self.current_pressed.clear()
        return self.curr_joint_actions
