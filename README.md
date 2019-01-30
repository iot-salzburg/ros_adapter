# PandaCode

## Startanleitung für den PandaRoboter im Demonstrationsmodus:

#### Terminal 1 - Franka Control:
roslaunch franka_control franka_control.launch robot_ip:=192.168.13.1

#### Terminal 2 - MoveIt Trajektorienberechnung:
roslaunch panda_moveit_config panda_moveit.launch

#### Terminal 3 - Rviz:
roslaunch panda_moveit_config moveit_rviz.launch

#### Terminal 4 - Ansteuerung Roboter:
source ~/libfranka/ws_moveit/devel/setup.bash
cd ~/libfranka/ws_moveit/
rosrun niks_experiments Stretching

#### Terminal 5 - OPC UA Server & ROS Bridge:
source ~/libfranka/ws_moveit/devel/setup.bash
cd ~/libfranka/ws_moveit/
rosrun niks_experiments opc_ua_ros_server.py

#### RVIZ Fenster:
Im Rviz Fenster -> obere Leiste "Panel" -> Add New Panel -> RvizVisualToolsgui (OK Drücken, nicht doppelklick)
Dann unten auf "Continue"

## Nach Aufsetzen eines neuen Systems:

#### ROS Aufsetzen
#### Catkin Workspace erstellen
#### Im Workspacefolder src dieses git hier pullen
#### "catkin build" ausführen


## Bei Änderungen an Stretching.cpp 
#### im workspace folder oder package folder "catkin build" ausführen

