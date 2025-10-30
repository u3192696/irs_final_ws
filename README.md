# IRS Final Robot System Build Instructions

## Prerequisites

Before starting, ensure ROS2 Humble is installed and sourced in your Ubuntu 22.04 environment. You should also have Git and colcon build tools installed.

## Clone the GitHub Repository

Navigate to the directory where you want your ROS2 workspace, and clone the GitHub repository.

```bash
cd ~/ros2_ws/
git clone https://github.com/u3192696/irs_final_ws.git
```

## Install Dependencies

After cloning, install all required dependencies for the packages in your workspace.

```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```
This command checks for missing dependencies and installs them automatically.

## Source ROS2 Humble

Ensure your ROS2 Humble installation is sourced:

```bash
source /opt/ros/humble/setup.bash
```

## Build the Workspace

Use colcon to build all packages in your workspace:

```bash
cd ~/ros2_ws
colcon build
```

## Understanding the Build Output

After building, colcon creates three new directories alongside `src`:

- **build/**: Contains intermediate build files and CMake artifacts
- **install/**: Contains the installed packages ready for execution
- **log/**: Contains logging information about the build process

## Source the Workspace

After a successful build, source your workspace overlay:

```bash
source ~/ros2_ws/install/setup.bash
```

## Optional Steps
### Automate sourcing
You can add this line (`source ~/ros2_ws/install/setup.bash`) to your `~/.bashrc` file to automatically source it in new terminals.

### Verify the Build
Check that the packages are recognised:

```bash
ros2 pkg list
```

## Launch the ROS2 nodes:
In one terminal, launch the main hs_robot_system nodes.
```bash
ros2 launch hs_robot_system hs_robot_system.launch.py
```

In a seperate terminal, launch the arm_control nodes.
```bash
ros2 launch tm12x_moveit_config hand_solo_moveit.launch.py
```
