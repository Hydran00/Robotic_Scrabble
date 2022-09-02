# Kudos to DOROWU for his amazing VNC 16.04 KDE image
FROM dorowu/ubuntu-desktop-lxde-vnc:focal
LABEL maintainer "nardi&perantoni"

# Adding keys for ROS
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-get install curl
RUN wget -q -O - https://dl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Installing ROS
RUN sudo apt-get update -y
RUN sudo apt-get install ros-noetic-desktop-full -y
RUN bash /opt/ros/noetic/setup.bash 

# Install dependencies
RUN sudo apt-get install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
RUN sudo apt-get install python3-rosdep -y

# Creating ROS_WS
RUN /bin/bash -c "echo 'export HOME=/root/' >> /root/.bashrc && source /root/.bashrc"
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN mkdir /root/catkin_ws

# Set up the workspace
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash && \
                  cd /root/catkin_ws/"
RUN sudo apt-get install ros-noetic-catkin
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /root/catkin_ws/;'

# Installing git
RUN sudo apt-get update -y
RUN sudo apt-get install git -y
RUN cd /root/catkin_ws
RUN /bin/bash -c '. /root/.bashrc'

# Install MoveIt!
RUN /bin/bash -c 'cd /root/catkin_ws && git clone https://github.com/Hydran00/Robotic_Scrabble.git &&  mv Robotic_Scrabble src'
RUN sudo apt-get install ros-noetic-moveit -y
RUN sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers -y
RUN cd /root/catkin_ws
RUN sudo apt update -qq
RUN sudo rosdep init
RUN sudo apt-get install ros-noetic-ur-client-library
# Create gazebo models folder
RUN /bin/bash -c 'cd /root && mkdir .gazebo && cd .gazebo && mkdir models'
RUN mv /root/catkin_ws/src/laboratorio /root/.gazebo/models
RUN /bin/bash -c "echo 'export GAZEBO_MODEL_PATH=/root/.gazebo/models:$GAZEBO_MODEL_PATH' >> /root/.bashrc"

# Install required ur msgs
RUN /bin/bash -c "git clone https://github.com/UniversalRobots/Universal_Robots_ROS_scaled_controllers.git"
RUN /bin/bash -c "git -C /root/catkin_ws/src clone https://github.com/gavanderhoorn/industrial_robot_status_controller.git"
RUN /bin/bash -c "git clone https://github.com/ros-industrial/ur_msgs.git"
RUN /bin/bash -c "git clone https://github.com/UniversalRobots/Universal_Robots_ROS_passthrough_controllers"
RUN /bin/bash -c "git clone https://github.com/UniversalRobots/Universal_Robots_ROS_cartesian_control_msgs/"
RUN /bin/bash -c "git clone https://github.com/UniversalRobots/Universal_Robots_ROS_controllers_cartesian"
RUN rosdep update 
RUN rosdep install --from-paths /root/catkin_ws/src -i --rosdistro noetic -y
RUN /bin/bash -c 'cd /root/ && . /opt/ros/noetic/setup.bash && cd catkin_ws/ && catkin_make'

# Adding permissions
RUN cd /root/catkin_ws/src/scrabble/scripts &&  chmod +x main.py && chmod +x printBoard.py

RUN cd /root/catkin_ws/src/soft_robotics/soft_robotics_description/scripts && chmod +x gripper_controller.py 
		
# Sourcing
RUN /bin/bash -c ". /root/catkin_ws/devel/setup.bash"
RUN /bin/bash -c "echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc"


