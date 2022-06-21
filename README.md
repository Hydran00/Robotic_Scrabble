# Robotic_Scrabble
Ur5e playing scrabble against humans

**ROBOT REALE**

**Calibrazione(solo la prima volta):**

roslaunch ur_calibration calibration_correction.launch  robot_ip:=192.168.0.100 target_filename:=$(rospack find ur_calibration)/etc/ex-ur5e_calibration.yaml 

**Lancio configurazione laboratorio**

```
roslaunch ur5e_gripper_moveit_config ur5e_lab.launch
```

**Lancio nodo per invio di comandi al gripper:**

```
rosrun soft_robotics_description gripper_controller.py
```

######################

**SIMULAZIONE IN GAZEBO** 

```
roslaunch ur5e_gripper_moveit_config ur5e_gazebo.launch
```
######################

**Esecuzione demo**

```
rosrun motion_plan demo.cpp
```



https://user-images.githubusercontent.com/93198865/174913901-e2afb5cb-7508-4986-9b46-b6956a712838.mp4



https://user-images.githubusercontent.com/93198865/174914093-a0258b16-fbbb-4a10-b888-1c11401b6d9a.mp4


