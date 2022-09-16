# AUBO ROS DEMO

## Environment

- Ubuntu 18.04
- ROS Melodic

## Requirements

```bash
# ros, ros-industrie, libconfig
sudo apt install ros-melodic-industrial-core
sudo apt install ros-melodic-moveit   
sudo apt install libconfig-dev    

# gazebo
sudo apt-get install ros-melodic-transmission-interface
sudo apt-get install ros-melodic-gazebo-ros-control
sudo apt-get install ros-melodic-joint-state-controller
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-position-controllers
sudo apt-get install ros-melodic-moveit-visual-tools

# 编译 AUBO Repo
mkdir -p aubo_ros_demo/src
cd aubo_ros_demo/src
git clone -b melodic https://github.com/AuboRobot/aubo_robot

# 通过以下方法可以下载ros包相关依赖 
rosdep install --from-paths src --ignore-src -r -y

# 检查所有CMakeLists 中与melodic相关的内容
# 某些cmake中遗漏了melodic的检测, 可能导致Eigen3无法找到 !!

# 对‘vtable for google::protobuf::MessageLite’未定义的引用
# https://github.com/AuboRobot/aubo_robot/issues/73
sudo apt-get update
sudo apt-get install gdebi
wget http://archive.ubuntu.com/ubuntu/pool/main/p/protobuf/libprotobuf9v5_2.6.1-1.3_amd64.deb
sudo gdebi ./libprotobuf9v5_2.6.1-1.3_amd64.deb

# export LD_LIBRARY_PATH
# libconfig
export LD_LIBRARY_PATH=/home/shan/App/config:$LD_LIBRARY_PATH

# log4cplus
export LD_LIBRARY_PATH=/home/shan/App/log4cplus/linux_x64/lib:$LD_LIBRARY_PATH

# 编译
catkin_make
```

## Usage

- MoveIt + RVIZ

  ```bash
  # launch moveit + rviz
  roslaunch aubo_i5_moveit_config moveit_planning_execution.launch robot_ip:=127.0.0.1
  ```
- MoveIt + RVIZ + Gazebo

  ```bash
  # launch moveit + rviz
  roslaunch aubo_i5_moveit_config moveit_planning_execution.launch robot_ip:=127.0.0.1
  # launch gazebo
  roslaunch aubo_gazebo aubo_i5_gazebo_control.launch
  ```
- MoveIt + Arm

  ```bash
  # 网线连接, 手动设置IP 192.168.0.100
  # 如何确认设备IP?
  roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.0.123
  ```