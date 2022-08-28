/**
 * @file scrabble_robot_control.cpp
 * @author Davide Nardi
 * @brief Ros node that receive command from the scrabble_node and send motion command to the robot
 * @version 0.1
 * @date 2022-06-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <moveit/move_group_interface/move_group_interface.h>
#include "ros/ros.h"
#include <cstdlib>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "scrabble/PutTile.h"
#include <std_srvs/Trigger.h>

#define SIMULATION true
#define PI 3.14159
#define GRIPPER_MAX_CLOSURE 0.6
#define GRIPPER_MIN_CLOSURE 0.5
#define Z_BASE_LINK 1.79
#define Z_DESK 0.867
#define GRIPPER_LENGHT 0.2
#define TILE_NUM 30

#define TILE_STACK_POS_X 0.5
#define TILE_STACK_POS_Y 0
#define TILE_STACK_POS_Z Z_DESK + 0.1
#define TILE_STACK_ROT_X 0
#define TILE_STACK_ROT_Y 0
#define TILE_STACK_ROT_Z 0
#define TILE_STACK_ROT_W 1

#define BOARD_X -0.30423
#define BOARD_Y -0.27894
#define BOARD_Z 0.896
#define BOARD_LENGTH 0.429
#define CELL_LENGTH BOARD_LENGTH/17

#define RACK_POS_X 0.2
#define RACK_POS_Y 0.25
#define RACK_POS_Z 0.918
#define RACK_ROT_X 0
#define RACK_ROT_Y 0
#define RACK_ROT_Z 1
#define RACK_ROT_W 0

scrabble::PutTile msg1;
float RACK_X[7];
float RACK_Y[7];
bool put_tile_flag=false;
using namespace std;
using namespace Eigen;
using namespace ros;
using namespace boost;

//define moveit groups' names
static const std::string PLANNING_GROUP_ARM = "manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "endeffector";
moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::MoveGroupInterface *gripper_group;
moveit::planning_interface::MoveGroupInterface::Plan *arm_motion_plan;
moveit::planning_interface::MoveGroupInterface::Plan *gripper_plan;


geometry_msgs::Pose tile_stack,rack;
/**
 * @brief Initialize pointers and global variables
 * 
 */
void setup()
{
    //initialize groups and plans
    arm_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM);
    gripper_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER);
    arm_group->setPlanningTime(5.0);
    arm_group->setMaxVelocityScalingFactor(0.3);
    arm_group->setMaxAccelerationScalingFactor(0.1);
    arm_motion_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
    gripper_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
    tile_stack.position.x = TILE_STACK_POS_Y;
    tile_stack.position.y = TILE_STACK_POS_X;
    tile_stack.position.z = TILE_STACK_POS_Z;
    tile_stack.orientation.x = TILE_STACK_ROT_X;
    tile_stack.orientation.y = TILE_STACK_ROT_Y;
    tile_stack.orientation.z = TILE_STACK_ROT_Z;
    tile_stack.orientation.w = TILE_STACK_ROT_W;
    rack.position.x = RACK_POS_Y;
    rack.position.y = RACK_POS_X;
    rack.position.z =  RACK_POS_Z;
    rack.orientation.x = RACK_ROT_X;
    rack.orientation.y = RACK_ROT_Y;
    rack.orientation.z = RACK_ROT_Z;
    rack.orientation.w = RACK_ROT_W;

    for(int i = 0;i<7;i++){
        RACK_Y[i] = 0.20 + 0.035 * i+0.009;
        cout<<"rack_y:"<<RACK_Y[i]<<endl;
    }
    for(int i = 0;i<7;i++){
        RACK_X[i] = 0-0.009;
        cout<<"rack_x:"<<RACK_X[i]<<endl;
    }
}
void printRED(string s){
    cout<<"\033[1;92m"<<s<<"\033[0m\n";
}
/**
 * @brief Function that can be used to set robot target position at a predefined position
 * 
 * @param target set a target position for the EE
 * @param plan moveit move group planner
 */
void define_target_pos(geometry_msgs::Pose target,moveit::planning_interface::MoveGroupInterface::Plan plan){
    cout<<"\033[1;34mMoving towards:\033[0m\n"<<endl;
    cout<<"\033[1;34mx: "<<target.position.x<<"\033[0m"<<endl;
    cout<<"\033[1;34my: "<<target.position.y<<"\033[0m"<<endl;
    cout<<"\033[1;34mz: "<<target.position.z<<"\033[0m"<<endl;
    arm_group->setPoseTarget(target);
    bool success = (arm_group->plan(*arm_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
        printRED("Motion plan updated!");
    }else{
        printRED("Error: Motion plan failed!");
    }
}
/**
 * @brief Start motion if a target position was defined with 'define_target_position_ function
 * 
 */
void execute_arm_motion_plan(){
    cout<<endl<<"Motion plan is now executing!";
    arm_group->move();
}
/**
 * @brief open gripper sending "open1" msg to gripper_controller_node
 * 
 * @param pub Ros client that call gripper_controller_node for sending cmd to gripper
 * @param client Ros service client used to re-establish connection to robot after it drops due to msg sending
 */
void open_gripper1(ros::Publisher pub,ros::ServiceClient client){
    if(SIMULATION){
        gripper_group->setNamedTarget("open");
        bool success = (gripper_group->plan(*gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        gripper_group->move();
        cout<<"gripper"<<endl;
        return;
    }
    std_msgs::String msg;

    std::stringstream ss;
    ss << "open1";
    msg.data = ss.str();
    cout<<"Sending opening msg to gripper_cmd "<<msg.data.c_str();
    pub.publish(msg);
    ros::Duration(1.3).sleep();
    std_srvs::Trigger srv;
    client.call(srv);
}
/**
 * @brief open gripper sending "open2" msg to gripper_controller_node
 * 
 * @param pub Ros client that call gripper_controller_node for sending cmd to gripper
 * @param client Ros service client used to re-establish connection to robot after it drops due to msg sending
 */
void open_gripper2(ros::Publisher pub,ros::ServiceClient client){
    if(SIMULATION){
        gripper_group->setNamedTarget("open");
        bool success = (gripper_group->plan(*gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        gripper_group->move();
        cout<<"gripper"<<endl;
        return;
    }
    std_msgs::String msg;

    std::stringstream ss;
    ss << "open2";
    msg.data = ss.str();
    cout<<"Sending opening msg to gripper_cmd "<<msg.data.c_str();
    pub.publish(msg);
    ros::Duration(1.3).sleep();
    std_srvs::Trigger srv;
    client.call(srv);
}
/**
 * @brief close gripper sending "close" msg to gripper_controller_node
 * 
 * @param pub Ros client that call gripper_controller_node for sending cmd to gripper
 * @param client Ros service client used to re-establish connection to robot after it drops due to msg sending
 */
void close_gripper(ros::Publisher pub,ros::ServiceClient client){
    if(SIMULATION){
        gripper_group->setNamedTarget("close");
        bool success = (gripper_group->plan(*gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        gripper_group->move();
        cout<<"gripper"<<endl;
        return;
    }
    std_msgs::String msg;
    std::stringstream ss;
    ss << "close";
    msg.data = ss.str();
    cout<<"Sending closing msg to gripper_cmd "<<msg.data.c_str();
    pub.publish(msg);
    ros::Duration(1.3).sleep();
    std_srvs::Trigger srv;
    client.call(srv);
}
/**
 * @brief Compute the position of a cell given row and column
 * 
 * @param row Row of the board
 * @param column Column of the board
 */
geometry_msgs::Pose getCellPosition(int row,int column){
    geometry_msgs::Pose target;
    target.position.z = BOARD_Z;
    //calcolo la posizione della cella in alto a sinistra
    target.position.x = BOARD_X-(BOARD_LENGTH/2);
    target.position.y = BOARD_Y-(BOARD_LENGTH/2);
    //calcolo la posizione del centro della cella (CELL_LENGTH = BOARD_X/17)
    target.position.x = target.position.x  + CELL_LENGTH*(column-1)+ CELL_LENGTH/2;
    target.position.y = target.position.y  + CELL_LENGTH*((17-row)-1) +CELL_LENGTH/2;
    //imposto quaternione per la rotazione dell'ee
    target.orientation.x=target.orientation.w=target.orientation.y=0;
    target.orientation.z=1;
    return target;
}

/**
 * @brief Execute cartesian path given the target position. If no cartesian solution are found, function return immediately otherwise EE will start moving
 * 
 * @param target desired target pose (X,Y,Z,x,y,z,w) of EE 
 */
void execute_Cartesian_Path(geometry_msgs::Pose target){
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose current_pose = (arm_group->getCurrentPose()).pose;
    waypoints.push_back(target);
    cout<<"\033[1;34mMoving towards:\033[0m\n"<<endl;
    cout<<"\033[1;34mx: "<<target.position.x<<"\033[0m"<<endl;
    cout<<"\033[1;34my: "<<target.position.y<<"\033[0m"<<endl;
    cout<<"\033[1;34mz: "<<target.position.z<<"\033[0m"<<endl;
    cout<<"\033[1;34mz: "<<target.orientation.z<<"\033[0m"<<endl;
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = arm_group->computeCartesianPath(waypoints,0.01,0,trajectory);
    cout<<"Fraction: "<<fraction<<endl; 
    if(fraction<1){
        exit(1);
    }
    int sz_tr = trajectory.joint_trajectory.points.size();
    float duration = 0.1;
    for (int i=1;i<sz_tr;i++){
        ros::Duration time_dur_ros = ros::Duration(duration);
        trajectory.joint_trajectory.points[i].time_from_start = time_dur_ros;
        duration += 0.05;

    }
    if(fraction==1){
        arm_motion_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
        arm_motion_plan->trajectory_ = trajectory;
        arm_group->execute(*arm_motion_plan);
        cout<<"executing motion..."<<endl;
    }        
    ros::Duration(0.3).sleep();
}
/**
 * @brief Populate information about desired position of letters in the rack and their correspondents row and column.
 * This will be called when scrabble_node send msg for putting a word on the board.
 * 
 * @param msg Msg that come from scrabble_node
 */
void put_tile_callback(const scrabble::PutTile::ConstPtr &msg){
    for(int i=0;i<7;i++){
        msg1.rack_pos[i] = msg->rack_pos[i];
        msg1.target_col[i] = msg->target_col[i];
        msg1.target_row[i] = msg->target_row[i];
    }
   
    put_tile_flag = true;
}
/**
 * @brief Motion plan for putting a word on board one tile at the time
 * 
 * @param client Ros client that call gripper_controller_node for sending cmd to gripper
 * @param gripper_pub 
 */
void put_tile(ros::ServiceClient client,ros::Publisher gripper_pub){
    geometry_msgs::Pose target;
        for(int i=0;i<7;i++){
        //come dicevo al cap. 7.5, valuto solo rack_pos != 0
        if(msg1.rack_pos[i]==0){
            continue;
        }
        //definisco target come la posizione della prima tessera da spostare (vedi Fig 7.2)
        target.position.x = RACK_X[msg1.rack_pos[i]-1];
        target.position.y = RACK_Y[msg1.rack_pos[i]-1]; 
        target.position.z = 0.86;
        //imposto l'orientamento (tubo dell'aria verso il muro) usando i quaternioni
        target.orientation.x=target.orientation.w=target.orientation.y=0;
        target.orientation.z=1;
        //mi muovo sopra la tessera del leggio
        execute_Cartesian_Path(target);
        //mantengo lo stesso target ma cambio l'altezza
        //il tavolo non è perfettamente in bolla, quindi l'altezza è parametrizzata rispetto alla posizione della tessera
        target.position.z = RACK_POS_Z - (0.003/7)*msg1.rack_pos[i];
        //eseguo il nuovo movimento (il gripper si allinea con la tessera)        
        execute_Cartesian_Path(target);
        //gripping della tessera
        close_gripper(gripper_pub,client);
        //alzo l'end effector
        target.position.z = 0.86;;
        execute_Cartesian_Path(target);
        //calcolo la posizione nel frame del robot della cella target usando riga e colonna che arrivano dal messaggio
        target = getCellPosition(msg1.target_row[i],msg1.target_col[i]);
		//risultato della funzione ha valore y opposto      
        target.position.y = - target.position.y;
        target.position.z = 0.86;
        //muovo il robot sopra la cella target
        execute_Cartesian_Path(target);
        //abbasso l'end effector modificando solo il valore sull'asse z
        target.position.z = 0.907;
        execute_Cartesian_Path(target); 
        //rilascio tessera        
        open_gripper1(gripper_pub,client);
        //alzo nuovamente l'end effector
        target.position.z = 0.80;
        execute_Cartesian_Path(target);
        //apro completamente le dita
        open_gripper2(gripper_pub,client);
    }
}

/////////////////////////

int main(int argc,char** args){
    ros::init(argc, args, "move_group_interface");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    ros::Publisher gripper_pub = n.advertise<std_msgs::String>("gripper_controller_cmd", 10);
    spinner.start();
    setup();
    ros::ServiceClient client;
    if(!SIMULATION){
        client = n.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/resend_robot_program");
    }
    ros::Subscriber sub = n.subscribe("/scrabble/put_tile_on_board_command", 1, put_tile_callback);
    printRED("Subscribed and ready to receive command");
        
    while (ros::ok())
    {
        if(put_tile_flag){
            put_tile(client,gripper_pub);
            put_tile_flag=false;
        }
    }
}
