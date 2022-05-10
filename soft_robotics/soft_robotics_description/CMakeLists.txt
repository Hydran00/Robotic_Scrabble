cmake_minimum_required(VERSION 2.8.3)
project(soft_robotics_description)

find_package(
  catkin REQUIRED COMPONENTS
)

catkin_package()

 catkin_install_python(PROGRAMS
   scripts/gripper_controller.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
