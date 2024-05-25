# Jetson
Jetson ros for nav robot
## Get start
Go to youe Workspace
```
cd ~/worksp/--Your_WP--/src
git clone https://github.com/ChollatisP/Jetson.git
git checkout gazebo
cd ~/worksp/--Your_WP--
catkin_make
```
Then install pkg
```
sudo apt install ros-noetic-slam-toolbox
sudo apt install ros-noetic-slam-toolbox-rviz
sudo apt install ros-noetic-teb-local-planner
```
## MAP
Create map you have to run 
```
roslaunch my_mobile_robot slam_tool_rviz_gazebo.launch
```
```
cd my_mobile_robot/script
python3 pub_scan.py
```
After finish runnung for create map go to PANEL in RVIZ an click on Add new panel then type name of map at save map an serialize map
### How to use map
go to .ros dir an your map is here then move every files that hav same name as your map to ~/worksp/--YOUR_WS--/src/Jetson/my_mobile_robot/maps
#### Change config and Yaml file
go to yaml of your map and then change path of your pgm file and change map name in mapper_parmas_online_async_local.yaml
## Navigation
```
cd ~/worksp/--Your_WS--/src/Jetson/my_mobile_robot/script
roslaunch my_mobile_robot slam_tool_nav_teb_gazebo.launch
python3 pub_scan.py
```
