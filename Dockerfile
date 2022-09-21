FROM ubuntu:focal
LABEL maintainer "nardi&perantoni"
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update

# Adding keys for ROS
RUN apt-get update -y && apt-get install -y lsb-release && apt-get clean all
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get install curl -y && apt-get install wget -y
RUN apt-get install -y curl gnupg gnupg2
RUN wget -q -O - https://dl.google.com/linux/linux_signing_key.pub |  apt-key add -
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc |  apt-key add -

# Installing ROS
RUN  apt-get update && apt-get install ros-noetic-desktop-full -y
RUN bash /opt/ros/noetic/setup.bash 

# Install dependencies
RUN apt-get install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
RUN apt-get install python3-rosdep -y

# Creating ROS_WS
RUN /bin/bash -c "echo 'export HOME=/root/' >> /root/.bashrc && source /root/.bashrc"
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN mkdir /root/catkin_ws

# Set up the workspace
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash && \
                  cd /root/catkin_ws/"
RUN apt-get install ros-noetic-catkin
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /root/catkin_ws/;'

# Installing git
RUN  apt-get update -y
RUN  apt-get install git -y
RUN cd /root/catkin_ws
RUN /bin/bash -c '. /root/.bashrc'

# Install MoveIt!
RUN cd ..
RUN /bin/bash -c 'cd /root/catkin_ws && git clone https://github.com/Hydran00/Robotic_Scrabble.git &&  mv Robotic_Scrabble src'
RUN  apt-get install ros-noetic-moveit -y
RUN  apt-get install ros-noetic-ros-control ros-noetic-ros-controllers -y
RUN cd /root/catkin_ws
RUN  apt update -qq
RUN  rosdep init
RUN  apt-get install ros-noetic-ur-client-library
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
RUN /bin/bash -c 'cd /root/ && . /opt/ros/noetic/setup.bash && cd catkin_ws/ && (catkin_make || catkin_make)'

# Adding permissions
RUN cd /root/catkin_ws/src/scrabble/scripts &&  chmod +x main.py && chmod +x printBoard.py

RUN cd /root/catkin_ws/src/soft_robotics/soft_robotics_description/scripts && chmod +x gripper_controller.py 
		
# Sourcing
RUN /bin/bash -c ". /root/catkin_ws/devel/setup.bash"
RUN /bin/bash -c "echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc"
RUN  apt-get install pip -y
RUN pip install dawg texttable
RUN apt-get install gedit -y

# to build
#  docker build --rm  --tag ros1_ur5e_base . --file Dokerfile

# to run
#  xhost +
#  docker run -v /tmp/.X11-unix/:/tmp/.X11-unix/ --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --network=host --name ubuntu_bash --env="DISPLAY" --rm -i -t --privileged ros1_ur5e_base bash
