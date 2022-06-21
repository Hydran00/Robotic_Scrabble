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
