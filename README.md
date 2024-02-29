# mocap_optitrack_driver

## dependencies
Download dependencies repo:
```
git clone https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack.git
```
Install dependencies:
```
sudo apt install python3-vcstool
vcs import < mocap4ros2_optitrack/dependency_repos.repos
rm -rf mocap4ros2_optitrack
```
Compiling workspace:
```
cd .. && colcon build --symlink-install
```
Source workspace:
```
source install/setup.bash
```
## Serial Repo
Download Serial repo:
```
git clone -b arm --single-branch https://github.com/DGiun/optitrack_serial.git
```
Build Package
```
cd ~/ros2_ws
colcon build --packages-select optitrack_serial
```
Source workspace:
```
source install/setup.bash
```
if Setup your optitrack configuration:
```
/src/optitrack_serial/mocap_optitrack_driver/config/mocap_optitrack_driver_params.yaml
```
Permission of Serial USB
```
sudo chmod 666 /dev/ttyUSB0
```
Launch optitrack system:
```
ros2 launch optitrack_serial receiver.launch.py 
```
## Error
if error by tf2_geometry_msg
```
gedit ~/ros2_ws/src/mocap4r2/mocap4r2_robot_gt/mocap4r2_robot_gt/src/mocap4r2_robot_gt/gt_component.cpp 
```
Change Line20
```
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
```
