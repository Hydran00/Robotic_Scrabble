
# Docker
Possibilità di usare docker facendo il build dell'immagine usando il docker file. 

```
 docker build - < Dockerfile
```
Poi lanciare il container con 
```
docker run -ti --rm -p 6080:80 <id_docker_img>
```
Per visualizzare la grafica, aprire il browser ed inserire come url 
```
localhost:6080
```

# Installazione completa
Usare docker non permette di usare la scheda video (non ho provato nvidia-docker) e si ottengono basse prestazioni nella simulazione. Consiglio di installare tutto sul proprio computer. Per far funzionare tutto si possono seguire i passaggi che sono presenti nel dockerfile presente nella repo. Riassumendo

Io ho usato Ubuntu 20.04

-Installare ROS Noetic e catkin

-Installare ROS_controllers con

```
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```
-Installare MoveIt!

-Preparare l'ambiente catkin con
```
mkdir catkin_ws && cd catkin_ws
mkdir src && catkin_init_workspace
cd ..
catkin_make
```
-Installare tutti i messaggi della Universal Robot in una cartella (di seguito ho usato il path /root/catkin_ws/src)
```
rosdep init
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_scaled_controllers.git
git clone https://github.com/gavanderhoorn/industrial_robot_status_controller.git
git clone https://github.com/ros-industrial/ur_msgs.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_passthrough_controllers
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_cartesian_control_msgs/
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_controllers_cartesian
rosdep update 
rosdep install --from-paths /root/catkin_ws/src -i --rosdistro noetic -y
```
-Clonare la repository dentro catkin_ws e rinominare la cartella clonata in src
```
cd catkin_ws/
git clone https://github.com/Hydran00/Robotic_Scrabble.git
mv Robotic_Scrabble src
```
-Build del progetto
```
catkin_make
```
-spostare il modello 3d del tavolo nella cartella dei modelli di gazebo
```
mv /root/catkin_ws/src/laboratorio /root/.gazebo/models
```

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

**Iniziare una partita**

```
rosrun motion_plan scrabble_robot_control_node
```
ed in un altro terminale
```
rosrun scrabble main.py
```


# DEMO
https://user-images.githubusercontent.com/93198865/174913901-e2afb5cb-7508-4986-9b46-b6956a712838.mp4



https://user-images.githubusercontent.com/93198865/174914093-a0258b16-fbbb-4a10-b888-1c11401b6d9a.mp4


