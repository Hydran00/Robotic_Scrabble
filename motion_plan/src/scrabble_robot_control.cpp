#include<iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ros/ros.h"
#include <cstdlib>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/String.h>
#include "motion_plan/Attach.h"
#include "scrabble/PutTile.h"
#include <std_srvs/Trigger.h>

#define HOME_X 
#define HOME_Y
#define HOME_Z



#define SIMULATION false
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

#define BOARD_X -0.30423//0.14362//-0.34//0.2
#define BOARD_Y -0.27894//-0.10823//-0.27//-0.25
#define BOARD_Z 0.896//0.913
#define BOARD_LENGTH 0.429
#define CELL_LENGTH BOARD_LENGTH/17

#define RACK_POS_X 0.2
#define RACK_POS_Y 0.25
#define RACK_POS_Z 0.916//Z_DESK + 0.01
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
static const std::string PLANNING_GROUP_GRIPPER = "gripper_group";
//pointer to moving group and objects
typedef struct Tile{
    std::string letter="_";
    geometry_msgs::Pose pose;
    void copy(geometry_msgs::Pose p){
        pose.position.x = p.position.x;
        pose.position.y = p.position.y;
        pose.position.z = p.position.z;
        pose.orientation.x = p.orientation.x;
        pose.orientation.y = p.orientation.y;
        pose.orientation.z = p.orientation.z;
        pose.orientation.w = p.orientation.w;
    }
}Tile;
Tile TILES[TILE_NUM];

moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::MoveGroupInterface *gripper_group;
moveit::planning_interface::MoveGroupInterface::Plan *arm_motion_plan;
moveit::planning_interface::MoveGroupInterface::Plan *gripper_plan;

motion_plan::Attach *attach_req, *detach_req;
ros::ServiceClient *attach_srvp, *detach_srvp,attach_srv,detach_srv;

geometry_msgs::Pose tile_stack,rack;

void setup()
{
    //initialize groups and plans
    arm_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM);
    //arm_group->setPlannerId("AnytimePathShortening");
    arm_group->setPlanningTime(5.0);
    arm_group->setMaxVelocityScalingFactor(0.3);
    arm_group->setMaxAccelerationScalingFactor(0.1);
    //gripper_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER);
    //gripper_group->setPlanningTime(1);
    //gripper_group->setMaxVelocityScalingFactor(1);
    //gripper_group->setMaxAccelerationScalingFactor(1);
    arm_motion_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
    //gripper_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
    attach_srvp = &attach_srv;
    detach_srvp = &detach_srv;
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
    }/*
    for(int i = 4;i<7;i++){
        RACK_Y[i] = 0.30 + 0.035 * (i-3)+0.009;
        cout<<"rack_y:"<<RACK_Y[i]<<endl;
    }*/
    for(int i = 0;i<7;i++){
        RACK_X[i] = 0-0.009;
        cout<<"rack_x:"<<RACK_X[i]<<endl;
    }/*
    for(int i = 4;i<7;i++){
        RACK_X[i] = 0.07-0.009;
        cout<<"rack_x:"<<RACK_X[i]<<endl;
    }*/

}
void printRED(string s){
    cout<<"\033[1;92m"<<s<<"\033[0m\n";
}


geometry_msgs::Pose normalize(geometry_msgs::Pose pose){
    return pose;
}

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

void execute_arm_motion_plan(){
    cout<<endl<<"Motion plan is now executing!";
    arm_group->move();
}

void open_gripper1(ros::Publisher pub,ros::ServiceClient open_client){
    //gripper_group->setJointValueTarget(gripper_group->getNamedTargetValues("closed"));
    //bool success = (gripper_group->plan(*gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //printRED("Opening gripper");
    //gripper_group->move();
    if(SIMULATION){
        return;
    }
    std_msgs::String msg;

    std::stringstream ss;
    ss << "open1";
    msg.data = ss.str();
    ROS_INFO("Sending opening msg to gripper_cmd", msg.data.c_str());
    pub.publish(msg);
    ros::Duration(1.3).sleep();
    std_srvs::Trigger srv;
    open_client.call(srv);
}
void open_gripper2(ros::Publisher pub,ros::ServiceClient open_client){
    if(SIMULATION){
        return;
    }
    std_msgs::String msg;

    std::stringstream ss;
    ss << "open2";
    msg.data = ss.str();
    ROS_INFO("Sending opening msg to gripper_cmd", msg.data.c_str());
    pub.publish(msg);
    ros::Duration(1.3).sleep();
    std_srvs::Trigger srv;
    open_client.call(srv);
}

void close_gripper(ros::Publisher pub,ros::ServiceClient close_client){
    //gripper_group->setJointValueTarget(gripper_group->getNamedTargetValues("home"));
    //bool success = (gripper_group->plan(*gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //printRED("Closing gripper");
    //gripper_group->move();
    if(SIMULATION){
        return;
    }
    std_msgs::String msg;
    std::stringstream ss;
    ss << "close";
    msg.data = ss.str();
    ROS_INFO("Sending closing msg to gripper_cmd", msg.data.c_str());
    pub.publish(msg);
    ros::Duration(1.3).sleep();
    std_srvs::Trigger srv;
    close_client.call(srv);
}

geometry_msgs::Pose getCellPosition(int row,int column){
    geometry_msgs::Pose target;
    cout<<"Row : "<<row<<", Column : "<< column;
    target.orientation.w = 0;
    target.orientation.x = 0;
    target.orientation.y = 0;
    target.orientation.z = 1;
    target.position.z = BOARD_Z;
    //get up-left corner
    target.position.x = BOARD_X-(BOARD_LENGTH/2);
    target.position.y = BOARD_Y-(BOARD_LENGTH/2);
    //get cell position
    target.position.x = target.position.x  + CELL_LENGTH*(column-1)+ CELL_LENGTH/2;
    target.position.y = target.position.y  + CELL_LENGTH*(row-1) +CELL_LENGTH/2;
    return target;
}


void execute_Cartesian_Path(geometry_msgs::Pose target){
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose current_pose = (arm_group->getCurrentPose()).pose;
    //waypoints.push_back(current_pose);
    waypoints.push_back(target);
    cout<<"\033[1;34mMoving towards:\033[0m\n"<<endl;
    cout<<"\033[1;34mx: "<<target.position.x<<"\033[0m"<<endl;
    cout<<"\033[1;34my: "<<target.position.y<<"\033[0m"<<endl;
    cout<<"\033[1;34mz: "<<target.position.z<<"\033[0m"<<endl;
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = arm_group->computeCartesianPath(waypoints,0.01,0,trajectory);
    cout<<"Fraction: "<<fraction<<endl; 
    if(fraction<1){
        exit(1);
    }
    int sz_tr = trajectory.joint_trajectory.points.size();
    float duration = 0.1;
    //trajectory.joint_trajectory.points[0].time_from_start= ros::Duration(0.01);
    for (int i=1;i<sz_tr;i++){
        ros::Duration time_dur_ros = ros::Duration(duration);
        trajectory.joint_trajectory.points[i].time_from_start = time_dur_ros;
        //cout<<"Time: "<<trajectory.joint_trajectory.points[i].time_from_start<<endl;
        duration += 0.05;//(-0.5*(cos(2*3.14*(i)/sz_tr+3.14))+0.5)*0.1;
        //cout<<"duration:"<<duration<<endl;
    }
    if(fraction==1){
        arm_motion_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
        arm_motion_plan->trajectory_ = trajectory;
        arm_group->execute(*arm_motion_plan);
        cout<<"execute";
    }     
    
}

void put_tile_callback(const scrabble::PutTile::ConstPtr &msg){
    for(int i=0;i<7;i++){
        msg1.rack_pos[i] = msg->rack_pos[i];
        msg1.target_col[i] = msg->target_col[i];
        msg1.target_row[i] = msg->target_row[i];
    }
   
    put_tile_flag = true;
}
void put_tile(ros::ServiceClient client,ros::Publisher gripper_pub){
    geometry_msgs::Pose target;
    int random_row,random_column;
    int i=0;
    for(int i=0;i<7;i++){
        if(msg1.rack_pos[i]==0){
            continue;
        }
        //pick up tile
        target.position.x = RACK_X[msg1.rack_pos[i]-1];
        cout<<"x::"<< target.position.x;
        target.position.y = RACK_Y[msg1.rack_pos[i]-1]; 
        cout<<"y::"<< target.position.y;
        target.position.z = 0.86;//RACK_POS_Z-0.1; 
        target.orientation.x=target.orientation.w=target.orientation.y=0;
        target.orientation.z=1;
        execute_Cartesian_Path(target);
        ros::Duration(0.1).sleep();
        target.position.z = RACK_POS_Z;
        execute_Cartesian_Path(target);
        ros::Duration(0.1).sleep();
        close_gripper(gripper_pub,client);
        target.position.z = 0.86;;
        execute_Cartesian_Path(target);
        ros::Duration(0.1).sleep();
        //drop tile
        target = getCellPosition(msg1.target_row[i],msg1.target_col[i]);
        target = normalize(target);
        target.position.y = - target.position.y;
        target.position.z = 0.86;
        execute_Cartesian_Path(target);
        ros::Duration(0.1).sleep();
        target.position.z = 0.907;
        // target.position.z = 0.907- 0.0073*(17-msg1.target_col[i])/17;
        execute_Cartesian_Path(target);
        open_gripper1(gripper_pub,client);
        target.position.z = 0.86;
        execute_Cartesian_Path(target);
        ros::Duration(0.1).sleep();
        open_gripper2(gripper_pub,client);
        ros::Duration(0.1).sleep();
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
    //Going to home position
    //arm_group->setJointValueTarget(arm_group->getNamedTargetValues("home"));
    //bool success = (arm_group->plan(*arm_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //printRED("Closing gripper");
    //arm_group->move();
    //attach_srv = n.serviceClient<motion_plan::Attach>("/link_attacher_node/attach");
    //detach_srv =  n.serviceClient<motion_plan::Attach>("/link_attacher_node/detach");
    ros::ServiceClient client;
    if(!SIMULATION){
        client = n.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/resend_robot_program");
    }
    rack = normalize(rack);
    tile_stack= normalize(tile_stack);
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
