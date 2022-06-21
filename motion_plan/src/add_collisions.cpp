/**
 * @file add_collisions.cpp
 * @author Davide Nardi
 * @brief Ros node that add laboratory's collisions to moveit environment
 * @version 0.1
 * @date 2022-06-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include<iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ros/ros.h"
#include <cstdlib>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include<signal.h>

#define Z_BASE_LINK 1.79
#define Z_DESK 0.867

using namespace std;
using namespace ros;


static const std::string PLANNING_GROUP_ARM = "manipulator"; // define moveit planning group name


moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
moveit_msgs::CollisionObject *desk;
moveit_msgs::CollisionObject *electric_panel;
moveit_msgs::CollisionObject *wall;
moveit_msgs::CollisionObject *top_plate;
moveit_msgs::CollisionObject *plug;

void printRED(string s){
    cout<<"\033[1;92m"<<s<<"\033[0m\n";
}

/**
 * @brief Function that load collision into the planning scene
 * 
 */
void addCollisions(){
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    // Collision object
    desk = new moveit_msgs::CollisionObject();
    electric_panel = new moveit_msgs::CollisionObject();
    wall = new moveit_msgs::CollisionObject();
    top_plate = new moveit_msgs::CollisionObject();
    plug = new moveit_msgs::CollisionObject();

    desk->header.frame_id = arm_group->getPlanningFrame();
    electric_panel->header.frame_id = arm_group->getPlanningFrame();
    wall->header.frame_id = arm_group->getPlanningFrame();
    top_plate->header.frame_id = arm_group->getPlanningFrame();
    plug->header.frame_id = arm_group->getPlanningFrame();

    desk->id = "desk";
    electric_panel->id= "electric_panel";
    wall->id= "wall";
    top_plate->id = "top_plate";

    shape_msgs::SolidPrimitive primitive1,primitive2,primitive3,primitive4,primitive5;
    //defining desk's collision
    primitive1.type = primitive1.BOX;
    primitive1.dimensions.resize(3);
    primitive1.dimensions[0] = 0.85;
    primitive1.dimensions[1] = 2.5;
    primitive1.dimensions[2] = 0.8675;
    //defining electric panel's collision
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 0.175;
    primitive2.dimensions[1] = 1;
    primitive2.dimensions[2] = 0.175;
    //defining wall's collision
    primitive3.type = primitive3.BOX;
    primitive3.dimensions.resize(3);
    primitive3.dimensions[0] = 0.05;
    primitive3.dimensions[1] = 1;
    primitive3.dimensions[2] = 2;
    //defining top plate's collision
    primitive4.type = primitive4.BOX;
    //primitive4.type = primitive3.BOX;
    primitive4.dimensions.resize(3);
    primitive4.dimensions[0] = 0.5;
    primitive4.dimensions[1] = 1;
    primitive4.dimensions[2] = 0.05;
    //defining electric plug's collision
    primitive5.type = primitive5.BOX;
    primitive5.dimensions.resize(3);
    primitive5.dimensions[0] = 0.1;
    primitive5.dimensions[1] = 0.12;
    primitive5.dimensions[2] = 0.09;
    
    geometry_msgs::Pose desk_pose,electric_panel_pose,wall_pose,top_plate_pose,plug_pose;
    //defining desk's pose
    desk_pose.orientation.w = -0.707;
    desk_pose.orientation.z = 0.707;
    desk_pose.position.x = -0.75;
    desk_pose.position.y = 0.08;
    desk_pose.position.z = Z_BASE_LINK-0.867/2;
    //defining electric panel's pose
    electric_panel_pose.orientation.w = -0.707;
    electric_panel_pose.orientation.z = 0.707;
    electric_panel_pose.position.x = 0;
    electric_panel_pose.position.y = -0.255;
    electric_panel_pose.position.z = 0.92-0.075;
    //defining wall's pose
    wall_pose.orientation.z = 0.707;
    wall_pose.orientation.w = -0.707;
    wall_pose.position.x = 0;
    wall_pose.position.y = -0.375;
    wall_pose.position.z = 1;
    //defining top plate's pose
    top_plate_pose.orientation.z =0.707;
    top_plate_pose.orientation.w =- 0.707;
    top_plate_pose.position.x = 0;
    top_plate_pose.position.y = -0.15;
    top_plate_pose.position.z = -0.026;
    //defining plug's pose
    plug_pose.orientation.z =0.707;
    plug_pose.orientation.w =- 0.707;
    plug_pose.position.x = 0.44;
    plug_pose.position.y = -0.2;
    plug_pose.position.z = 0.92-0.06;
    //push collisions into vector
    desk->primitives.push_back(primitive1);
    desk->primitive_poses.push_back(desk_pose);
    desk->operation = desk->ADD;

    electric_panel->primitives.push_back(primitive2);
    electric_panel->primitive_poses.push_back(electric_panel_pose);
    electric_panel->operation = electric_panel->ADD;

    wall->primitives.push_back(primitive3);
    wall->primitive_poses.push_back(wall_pose);
    wall->operation = wall->ADD;

    top_plate->primitives.push_back(primitive4);
    top_plate->primitive_poses.push_back(top_plate_pose);
    top_plate->operation = top_plate->ADD;

    plug->primitives.push_back(primitive5);
    plug->primitive_poses.push_back(plug_pose);
    plug->operation = plug->ADD;

    //adding collision to planning scene interface
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    
    collision_objects.push_back(*desk);
    collision_objects.push_back(*electric_panel);
    collision_objects.push_back(*wall);
    collision_objects.push_back(*top_plate);
    collision_objects.push_back(*plug);
    
    planning_scene_interface->applyCollisionObjects(collision_objects); //applyng collision to scene interface
}

/**
 * @brief Unload collision from moveit when node is stopped by signal
 * 
 * @param sig Signal number
 */
void removeCollision(int sig){
    cout<<endl<<"Deleting collisions";
    std::vector<std::string> object_ids;
    object_ids.push_back(desk->id);
    object_ids.push_back(electric_panel->id);
    object_ids.push_back(wall->id);
    object_ids.push_back(top_plate->id);
    object_ids.push_back(plug->id);
    planning_scene_interface->removeCollisionObjects(object_ids);
    ros::shutdown();
}



/////////////////////////

int main(int argc,char** args){
    ros::init(argc, args, "Collision_Node",ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, removeCollision);
    arm_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM);

    addCollisions();
    while (ros::ok()){
        ros::Duration(0.5).sleep();
    }
}
