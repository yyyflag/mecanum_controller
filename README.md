## Installation

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone https://github.com/yyyflag/yahboomcar_mecanum_controller.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select yahboomcar_mecanum_controller

# Source the workspace
source install/setup.bash

# testing
终端A:
ros2 launch yahboomcar_mecanum_controller mecanum.launch.py
终端B:
ros2 run yahboomcar_mecanum_controller mecanum_kinematics_node.py
终端C：
ros2 run teleop_twist_keyboard teleop_twist_keyboard
所有话题已就绪，机器人现在已经具备了完备的仿真感知能力，可以用于后续的建图（如SLAM）、导航算法测试等任务。
