/**
 * @file demo.cpp
 * @author Davide Nardi
 * @brief Demostration of motion plan with random movement of the robot above the board using gazebo simulation
 * @version 0.1
 * @date 2022-06-22
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ros/ros.h"
#include <cstdlib>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#define SIMULATION true
#define PI 3.14159
#define GRIPPER_MAX_CLOSURE 0.6
#define GRIPPER_MIN_CLOSURE 0.5
#define Z_BASE_LINK 1.79
#define Z_DESK 0.867
#define GRIPPER_LENGHT 0.2
#define TILE_NUM 30

#define BOARD_X -0.30423//0.14362//-0.34//0.2
#define BOARD_Y -0.27894//-0.10823//-0.27//-0.25
#define BOARD_Z 0.896//0.913
#define BOARD_LENGTH 0.429
#define CELL_LENGTH BOARD_LENGTH/17

#define RACK_POS_X 0.5
#define RACK_POS_Y 0.25
#define RACK_POS_Z Z_DESK + 0.01
#define RACK_ROT_X 0
#define RACK_ROT_Y 0
#define RACK_ROT_Z 0
#define RACK_ROT_W 1

using namespace std;
using namespace Eigen;
using namespace ros;
using namespace boost;

// define moveit groups' names
static const std::string PLANNING_GROUP_ARM = "manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "endeffector";
// pointer to moving group and objects
moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::MoveGroupInterface *gripper_group;
moveit::planning_interface::MoveGroupInterface::Plan *arm_motion_plan;
moveit::planning_interface::MoveGroupInterface::Plan *gripper_plan;


void setup()
{
    // initialize groups and plans
    arm_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM);
    // arm_group->setPlannerId("AnytimePathShortening");
    arm_group->setPlanningTime(5.0);
    arm_group->setMaxVelocityScalingFactor(0.3);
    arm_group->setMaxAccelerationScalingFactor(0.1);
    gripper_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER);
    gripper_group->setPlanningTime(1);
    gripper_group->setMaxVelocityScalingFactor(1);
    gripper_group->setMaxAccelerationScalingFactor(1);
    arm_motion_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
    gripper_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
}
void printRED(string s)
{
    cout << "\033[1;92m" << s << "\033[0m\n";
}

void execute_arm_motion_plan()
{
    cout << endl
         << "Motion plan is now executing!";
    arm_group->move();
}

void open_gripper()
{
    if (!SIMULATION) // with this it should work even with real robot
    {
        return;
    }
    gripper_group->setJointValueTarget(gripper_group->getNamedTargetValues("open"));
    bool success = (gripper_group->plan(*gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    printRED("Opening gripper");
    gripper_group->move();
}

void close_gripper()
{
    if (!SIMULATION)
    {
        return;
    }
    gripper_group->setJointValueTarget(gripper_group->getNamedTargetValues("close"));
    bool success = (gripper_group->plan(*gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    printRED("Closing gripper");
    gripper_group->move();
}

geometry_msgs::Pose getCellPosition(int row, int column)
{
    geometry_msgs::Pose target;
    cout << "Row : " << row << ", Column : " << column;
    target.orientation.w = 0;
    target.orientation.x = 0;
    target.orientation.y = 0;
    target.orientation.z = 1;
    target.position.z = BOARD_Z;
    // get up-left corner
    target.position.x = BOARD_X - (BOARD_LENGTH / 2);
    target.position.y = BOARD_Y - (BOARD_LENGTH / 2);
    // get cell position
    target.position.x = target.position.x + CELL_LENGTH * (column - 1) + CELL_LENGTH / 2;
    target.position.y = target.position.y + CELL_LENGTH * ((17 - row) - 1) + CELL_LENGTH / 2;
    return target;
}

void execute_Cartesian_Path(geometry_msgs::Pose target)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose current_pose = (arm_group->getCurrentPose()).pose;
    waypoints.push_back(current_pose);
    waypoints.push_back(target);
    // cout<<"\033[1;34mMoving towards:\033[0m\n"<<endl;
    cout << "\033[1;34mx: " << target.position.x << "\033[0m" << endl;
    cout << "\033[1;34my: " << target.position.y << "\033[0m" << endl;
    cout << "\033[1;34mz: " << target.position.z << "\033[0m" << endl;
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = arm_group->computeCartesianPath(waypoints, 0.01, 0, trajectory);
    cout << "Fraction: " << fraction << endl;
    if (fraction < 1.0)
    {
        exit(1);
    }
    arm_motion_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
    int sz_tr = trajectory.joint_trajectory.points.size();
    float duration = 0.1;
    for (int i = 1; i < sz_tr; i++)
    {
        ros::Duration time_dur_ros = ros::Duration(duration);
        trajectory.joint_trajectory.points[i].time_from_start = time_dur_ros;
        // cout<<"Time: "<<trajectory.joint_trajectory.points[i].time_from_start<<endl;
        duration += 0.05;
    }
    arm_motion_plan->trajectory_ = trajectory;
    arm_group->execute(*arm_motion_plan);
}
/**
 * @brief Move above board with random combination of row and column
 *
 * @param n number of motions
 */
void move_random_on_board(int n)
{
    geometry_msgs::Pose target;
    srand(time(NULL));
    int random_row, random_column, temp;
    for (int i = 0; i < n; i++)
    {
        open_gripper();
        target.position.x = 0;
        target.position.y = 0.4;
        target.position.z = 0.85;
        target.orientation.x = target.orientation.y = target.orientation.w = 0;
        target.orientation.z = 1;
        // go above rack
        execute_Cartesian_Path(target);
        ros::Duration(0.2).sleep();
        target.position.z = 0.905;
        // take cell
        execute_Cartesian_Path(target);
        ros::Duration(0.2).sleep();
        close_gripper();
        random_row = 9;//rand() % 17 + 1;
        random_column = 9;//rand() % 17 + 1;
        target = getCellPosition(random_row, random_column);
        // target = normalize(target);
        target.position.y = -target.position.y;
        // go above cell
        target.position.z = 0.85;
        execute_Cartesian_Path(target);
        ros::Duration(0.2).sleep();
        target.position.z = 0.905;
        execute_Cartesian_Path(target);
        ros::Duration(0.2).sleep();
    }
}

/////////////////////////

int main(int argc, char **args)
{
    ros::init(argc, args, "move_group_interface");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);

    spinner.start();
    setup();
    ros::Duration(0.5).sleep();
    // Going to home position
    // arm_group->setJointValueTarget(arm_group->getNamedTargetValues("home"));
    // bool success = (arm_group->plan(*arm_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // printRED("Closing gripper");
    // arm_group->move();
    move_random_on_board(5);
}
