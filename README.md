# mocap_optitrack_driver

## dependencies
Download dependencies repo:
```
git clone https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack.git
```
Install dependencies:
```
vcs import < mocap4ros2_optitrack/dependency_repos.repos
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
git clone https://github.com/DGiun/optitrack_serial.git
```
Source workspace:
```
source install/setup.bash
```
if Setup your optitrack configuration:
```
/src/optitrack_serial/mocap_optitrack_driver/config/mocap_optitrack_driver_params.yaml
```
Launch optitrack system:
```
ros2 launch optitrack_serial optitrack2.launch.py
```
Check that Optitrack configuration works fine and is connected. As the driver node is a lifecycle node, you should transition to activate:
```
ros2 lifecycle set /mocap_optitrack_driver_node activate
```
Visualize in rViz:
```
ros2 launch mocap_marker_viz mocap_marker_viz.launch.py mocap_system:=optitrack
```
