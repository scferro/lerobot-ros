# LeRobot ROS2

This package provides a ROS2 integration for [LeRobot](https://github.com/huggingface/lerobot), allowing you to control real-world robots using state-of-the-art machine learning models.

## Installation

To install this package, you first need to have a working ROS2 installation. Then, you can install this package from PyPI:

```bash
pip install lerobot-ros2
```

## Usage

This package provides a `ROS2Robot` class that you can use to control your robot. Here's a simple example:

```python
from lerobot.common.robots.ros2 import ROS2Robot, ROS2Config

config = ROS2Config()
robot = ROS2Robot(config)

robot.connect()

# Your robot control code here

robot.disconnect()
```

For more detailed examples, please refer to the `lerobot` documentation.