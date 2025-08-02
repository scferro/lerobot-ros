# LeRobot ROS

This repository provides a generic ROS 2 interface for the [LeRobot](https://github.com/huggingface/lerobot) framework. It acts as a lightweight wrapper to connect any [ros2_control](https://control.ros.org/rolling/index.html) or [MoveIt](https://moveit.ai/) compatible robot with the `lerobot` ecosystem. This enables `lerobot` to be used on a wide variety of real-world robots running on ROS 2.

A gamepad teleoperator for 6-DoF end-effector control and a keyboard teleoperator for joint position control is also provided.

**Supported control modes:**

- Joint position with ros2_control
  - Using [joint_trajectory_controller](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
  - Using [position_controllers](https://control.ros.org/rolling/doc/ros2_controllers/position_controllers/doc/userdoc.html)
- End-effector velocity with MoveIt 2
  - Using [Moveit Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- Gripper Controller with ros2_control
  - Using [joint_trajectory_controller](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
  - Using [Gripper Action Controller](https://control.ros.org/jazzy/doc/ros2_controllers/gripper_controllers/doc/userdoc.html)

**Supported Lerobot scripts:** teleoperate, record, replay

## Prerequisites

### Software Requirements

Before getting started, ensure you have the following installed:

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) - This repo is only tested on Jazzy.
- If end-effector control is desired, then [MoveIt2](https://moveit.ai/install-moveit2/binary) needs to be installed

### Robot Setup

For joint position control, the robot can be configured with either of the following setups:

**Option 1: JointGroupPositionController (default)**

- `position_controllers/JointGroupPositionController` for the robot arm joints
- `position_controllers/GripperActionController` for the gripper
- `joint_state_broadcaster/JointStateBroadcaster` for joint state feedback

**Option 2: JointTrajectoryController**

- `joint_trajectory_controller/JointTrajectoryController` for the robot arm joints
- `position_controllers/GripperActionController` for the gripper
- `joint_state_broadcaster/JointStateBroadcaster` for joint state feedback

The controller type is configured via the `arm_use_trajectory` parameter in the `ROS2InterfaceConfig`.

For end-effector control, the robot should be configured with the same gripper and joint state feedback as above, and additionally:

- `joint_trajectory_controller/JointTrajectoryController` for robot arm control
- `moveit_servo` node can be setup for real-time control

See [AR4 ROS Driver](https://github.com/ycheng517/ar4_ros_driver) for an example of a MoveIt2-enabled robot that works with LeRobot.

## Installation

Create a python virtual environment compatible with your ROS version (i.e. Python 3.12 for Jazzy). Install lerobot and this package. For example:

```bash
# Create virtual env
conda create -y -n lerobot-ros python=3.12
# Install lerobot from your lerobot source directory
pip install -e ~/src/lerobot
# Install lerobot-ros
pip install -e .
```

## Configuration

Create a config class for your robot by sub-classing `ROS2Config`.
You may override joint names, gripper configurations, and other parameters as needed.
An example config class for joint velocity control may look like this:

```python
from dataclasses import dataclass, field
from lerobot.common.robots.config import RobotConfig
from lerobot.common.robots.config import ROS2Config, ROS2InterfaceConfig

@RobotConfig.register_subclass("my_ros2_robot")
@dataclass
class MyRobotConfig(ROS2Config):
    action_type: ActionType = ActionType.JOINT_VELOCITY

    ros2_interface: ROS2InterfaceConfig = field(
        default_factory=lambda: ROS2InterfaceConfig(
            base_link="base_link",
            arm_joint_names=[
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
            ],
            gripper_joint_name="gripper_joint",
            gripper_open_position=0.0,
            gripper_close_position=1.0,
            max_linear_velocity=0.05,  # m/s
            max_angular_velocity=0.25,  # rad/s
        )
    )
```

## Getting Started

A good way to get started is to teleoperate your robot.

### Step 1: Launch Your Robot

Start your ROS2-enabled robot stack. The exact commands depend on your robot model and configuration.

**Example for [Annin Robotics AR4](https://github.com/ycheng517/ar4_ros_driver):**

```bash
# Launch the robot driver and ros2_control controllers
ros2 launch annin_ar4_driver driver.launch.py ar_model:=mk1 calibrate:=True

# Launch MoveIt2 with Moveit Servo enabled
ros2 launch annin_ar4_moveit_config moveit.launch.py ar_model:=mk1 moveit_servo:=True
```

### Step 2: Run the scripts

Once your robot is launched and ready, you can use the provided scripts. For example, to teleoperate your robot:

```bash
source /opt/ros/jazzy/setup.bash
python scripts/teleoperate.py \
    --robot.type=my_ros2_robot \
    --robot.id=my_awesome_follower_arm \
    --teleop.type=gamepad_6dof \
    --teleop.id=my_awesome_leader_arm \
    --display_data=true
```

### Next Steps

Once you have teleoperation working, you can use all standard LeRobot features as usual:

- Incorporate cameras and other sensors using the LeRobot repo
- Use [record.py](.scripts/record.py) from this repo to collect demonstration datasets
- Use [replay.py](.scripts/replay.py) from this repo to test recorded trajectories
- Train policies on your robot's data using the LeRobot repo
