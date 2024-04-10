# needle_pantograph_ros2
ROS2 stack to use a 2-Dof pantograph used at the ICube laboratory for needle insertion simulation.

## Installation

### Prepare the environment

1) Install ROS2 Humble and the usual tools (Gazebo, Colcon, etc.)

2) Install EtherLab as specified in the [documentation of ethercat_driver_ros2](https://icube-robotics.github.io/ethercat_driver_ros2/).

> [!TIP]
> use the `ethercat slaves` CLI command to check that all is OK andf that you can see your device.

### Install this package

```bash
WS_PANTOGRAPH=~/dev/ws_pantograph_ros2/
mkdir -p $WS_PANTOGRAPH/src
cd $WS_PANTOGRAPH/src

# git clone https://github.com/ICube-Robotics/needle_pantograph_ros2.git
git clone https://github.com/BlackSnow-333/needle_pantograph_ros2.git
vcs import . < needle_pantograph_ros2/needle_pantograph_ros2.repos
rosdep install --ignore-src --from-paths . -y -r

cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/setup.bash
```

## Getting started

```bash
cd $WS_PANTOGRAPH
source install/setup.bash
```

```bash
# Mock hardware
ros2 launch pantograph_bringup pantograph.launch.py
```

```bash
# With the actual robot
sudo /etc/init.d/ethercat start  # start ETherLab daemon
ros2 launch pantograph_bringup pantograph.launch.py use_fake_hardware:=false
```
