# dual_arm_moveit

## Installation
```bash
mkdir -p ~/dual_arm_ws/src
cd ~/dual_arm_ws/src
git clone https://github.com/bi3ri/dual_arm_demo.git
git submodule update --init --recursive
catkin build && source ~/dual_arm_ws/devel/setup.bash

#run in two shells
roslaunch dual_arm_demo app.launch 
rosrun dual_arm_demo moveit_demo
```