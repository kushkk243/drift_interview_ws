# Drift Workspace

## Installation

### Prerequisites
- ROS 2 (Humble or later recommended)
- Python 3.8+
- Git

### Required Packages

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-nav2 \
    python3-colcon-common-extensions
```

### Required Python Packages
```bash
pip install "numpy<2" opencv-python
```
## Setup

1. Clone and build the workspace:
```bash
cd ~/drift_interview_ws
colcon build
source install/setup.bash
```

## Launch Simulation

```bash
ros2 launch robot_description gazebo_show.launch.py
```

## Run Navigation

```bash
ros2 launch robot_description navigation.py
```

## Quick Start

Run both simulation and navigation:
```bash
# Terminal 1
ros2 launch robot_description gazebo_show.launch.py

# Terminal 2
cd ~/drift_interview_ws
source install/setup.bash
ros2 launch robot_description navigation.py
```
