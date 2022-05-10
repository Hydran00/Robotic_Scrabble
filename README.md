# Robotic_Scrabble
Ur5e playing scrabble against humans


LANCIARE ROBOT IN LABORATORIO

roslaunch ur_calibration calibration_correction.launch  robot_ip:=192.168.0.100 target_filename:=$(rospack find ur_calibration)/etc/ex-ur5e_calibration.yaml 
  
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.100 kinematics_config:=$(rospack find ur_calibration)/etc/ex-ur5e_calibration.yaml
  
roslaunch ur5e_gripper_moveit_config ur5e_moveit_planning_execution.launch limited:=true

roslaunch ur5e_gripper_moveit_config moveit_rviz.launch config:=true

#carico collisioni

rosrun motion_plan add_collision_node

#lancio rosnode per il gripper

rosrun soft_robotics_description gripper_controller.py


#esecuzione del pick and place

rosrun motion_plan motion_plan_node




######################
GAZEBO SIMULATION (work in progress)

roslaunch ur_gazebo ur5e_bringup.launch

roslaunch ur5e_gripper_moveit_config ur5e_moveit_planning_execution.launch sim:=true

roslaunch ur5e_gripper_moveit_config moveit_rviz.launch config:=true
