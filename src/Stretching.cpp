//      _____         __        __                               ____                                        __
//     / ___/ ____ _ / /____   / /_   __  __ _____ ____ _       / __ \ ___   _____ ___   ____ _ _____ _____ / /_
//     \__ \ / __ `// //_  /  / __ \ / / / // ___// __ `/      / /_/ // _ \ / ___// _ \ / __ `// ___// ___// __ \
//    ___/ // /_/ // /  / /_ / /_/ // /_/ // /   / /_/ /      / _, _//  __/(__  )/  __// /_/ // /   / /__ / / / /
//   /____/ \__,_//_/  /___//_.___/ \__,_//_/    \__, /      /_/ |_| \___//____/ \___/ \__,_//_/    \___//_/ /_/
//                                              /____/
//   Salzburg Research ForschungsgesmbH
//
//  Dominik Hofer, Michaela Mühlberger, Armin Niedermueller

//  DTZ ROS Robot Demonstrator
//  The purpose of this program is to control the panda robot via ros
//  It gets its TODOs via a rostopic from the opcua panda server

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <control_msgs/GripperCommandAction.h>
#include <franka_gripper/franka_gripper.h>

#include <ros/ros.h>
#include <ros/console.h>

#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <cstdlib>

#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>


std::string global_moving{"false"};
std::string global_order_movement{"XX"};
std::string global_response{"none"};
std_msgs::String global_ros_response;
int global_order_pos{0};
int global_temperature;

namespace rvt = rviz_visual_tools;


////////////////////////////////////////////////  ROBOT ROS METHODS  ///////////////////////////////////////////////////

void chatterCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("I heard: [%s]", msg->data.c_str());

    // create and initialize a stringstream object with our string
    std::stringstream ss{msg->data.c_str()};

    // split the string into to arguments
    ss >> global_order_movement;    // PO
    ss >> global_order_pos;         // 3


}


void moveFunction(std::vector<double> joint_group_positions, const robot_state::JointModelGroup* joint_model_group,
                  moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed, Eigen::Affine3d text_pose){

    move_group->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");

    move_group->setMaxVelocityScalingFactor(speed);

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    move_group->move();

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToInitialPosition(const robot_state::JointModelGroup* joint_model_group,
                           moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.498139,		 // Joint 1 
        -0.520210,		 // Joint 2 
        +0.097301,		 // Joint 3 
        -2.814416,		 // Joint 4 
        +0.052662,		 // Joint 5 
        +2.351891,		 // Joint 6 
        +0.801563		 // Joint 7 
        };



            

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToPrinter(const robot_state::JointModelGroup* joint_model_group,
                   moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    // Fängt an das Objekt vom Drucker zu heben

    std::vector<double> joint_group_positions;

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    joint_group_positions =
        {
        -2.508229,		 // Joint 1 
        +0.656498,		 // Joint 2 
        -0.440398,		 // Joint 3 
        -1.554039,		 // Joint 4 
        +0.927285,		 // Joint 5 
        +2.087697,		 // Joint 6 
        -0.944861		 // Joint 7 
        };


    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    // Position direkt beim Drucker
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        -2.516322,		 // Joint 1 
        +1.032788,		 // Joint 2 
        -0.322207,		 // Joint 3 
        -1.286464,		 // Joint 4 
        +1.095004,		 // Joint 5 
        +2.246536,		 // Joint 6 
        -0.998115		 // Joint 7 
        };


    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveFromPrinter(const robot_state::JointModelGroup* joint_model_group,
                     moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    // Fängt an das Objekt vom Drucker zu heben

    // Erste Position
    std::vector<double> joint_group_positions;

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        -2.542509,		 // Joint 1 
        +0.990563,		 // Joint 2 
        -0.307504,		 // Joint 3 
        -1.285260,		 // Joint 4 
        +1.104788,		 // Joint 5 
        +2.166730,		 // Joint 6 
        -0.978925		 // Joint 7 
        };



    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        -2.509485,		 // Joint 1 
        +0.801865,		 // Joint 2 
        -0.324251,		 // Joint 3 
        -1.375549,		 // Joint 4 
        +1.003452,		 // Joint 5 
        +2.132986,		 // Joint 6 
        -0.893283		 // Joint 7 
        };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToOutput(const robot_state::JointModelGroup* joint_model_group,
                  moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){
    
    std::vector<double> joint_group_positions;

    // Position 50cm über Förderband
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +0.844795,		 // Joint 1 
        -0.384217,		 // Joint 2 
        +1.890267,		 // Joint 3 
        -1.501299,		 // Joint 4 
        +0.146468,		 // Joint 5 
        +1.830871,		 // Joint 6 
        +2.016475		 // Joint 7 
        };



    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    // Ablege-Position beim Förderband
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +0.857302,		 // Joint 1 
        -1.322565,		 // Joint 2 
        +2.104476,		 // Joint 3 
        -1.837629,		 // Joint 4 
        +0.304724,		 // Joint 5 
        +2.699032,		 // Joint 6 
        +2.333015		 // Joint 7 
        };







    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveFromOutput(const robot_state::JointModelGroup* joint_model_group,
                    moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    // Erste Position
    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +0.844795,		 // Joint 1 
        -0.384217,		 // Joint 2 
        +1.890267,		 // Joint 3 
        -1.501299,		 // Joint 4 
        +0.146468,		 // Joint 5 
        +1.830871,		 // Joint 6 
        +2.016475		 // Joint 7 
        };





    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToStorage(const robot_state::JointModelGroup* joint_model_group,
                   moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){ // Passt
// Hier ne Position um vor dem Regal zu "Schweben"
    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +2.031532,		 // Joint 1 
        +1.054223,		 // Joint 2 
        +2.168870,		 // Joint 3 
        -2.671940,		 // Joint 4 
        -0.792631,		 // Joint 5 
        +3.065037,		 // Joint 6 
        +0.802607		 // Joint 7 
        };




    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Alle 9 Funktionen, die für das Finden des richtigen Lagerplatzes gedacht sind, fahren folgendes Muster:
// *Von Regal schwebend zu Platz schwebend
// *Von Platz schwebend zu ablegen/aufnehmen
// *Von ablegen/aufnehmen zu Platz schwebend

bool findPlaceOne(const robot_state::JointModelGroup* joint_model_group,
                  moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.900263,		 // Joint 1 
        +0.559978,		 // Joint 2 
        +2.802849,		 // Joint 3 
        -2.067452,		 // Joint 4 
        -0.311117,		 // Joint 5 
        +2.489097,		 // Joint 6 
        +0.959915		 // Joint 7 
        };



    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.902461,		 // Joint 1 
        +0.410919,		 // Joint 2 
        +2.765295,		 // Joint 3 
        -2.040703,		 // Joint 4 
        -0.174810,		 // Joint 5 
        +2.600436,		 // Joint 6 
        +0.832778		 // Joint 7 
        };



    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}

bool leavePlaceOne(const robot_state::JointModelGroup* joint_model_group,
                   moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.900263,		 // Joint 1 
        +0.559978,		 // Joint 2 
        +2.802849,		 // Joint 3 
        -2.067452,		 // Joint 4 
        -0.311117,		 // Joint 5 
        +2.489097,		 // Joint 6 
        +0.959915		 // Joint 7 
        };


    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
    return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceTwo(const robot_state::JointModelGroup* joint_model_group,
                  moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.853758,		 // Joint 1 
        +0.514412,		 // Joint 2 
        +2.624540,		 // Joint 3 
        -1.938238,		 // Joint 4 
        -0.033951,		 // Joint 5 
        +2.403973,		 // Joint 6 
        +0.527461		 // Joint 7 
        };


    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.841644,		 // Joint 1 
        +0.319584,		 // Joint 2 
        +2.607986,		 // Joint 3 
        -1.886534,		 // Joint 4 
        +0.033933,		 // Joint 5 
        +2.464868,		 // Joint 6 
        +0.527462		 // Joint 7 
        };



    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}

bool leavePlaceTwo(const robot_state::JointModelGroup* joint_model_group,
                   moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.853758,		 // Joint 1 
        +0.514412,		 // Joint 2 
        +2.624540,		 // Joint 3 
        -1.938238,		 // Joint 4 
        -0.033951,		 // Joint 5 
        +2.403973,		 // Joint 6 
        +0.527461		 // Joint 7 
        };


    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceThree(const robot_state::JointModelGroup* joint_model_group,
                    moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.793602,		 // Joint 1 
        +0.293711,		 // Joint 2 
        +2.425398,		 // Joint 3 
        -1.710999,		 // Joint 4 
        +0.358704,		 // Joint 5 
        +2.326216,		 // Joint 6 
        +0.096573		 // Joint 7 
        };


    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.785067,		 // Joint 1 
        +0.111577,		 // Joint 2 
        +2.395085,		 // Joint 3 
        -1.661979,		 // Joint 4 
        +0.492932,		 // Joint 5 
        +2.331837,		 // Joint 6 
        +0.039065		 // Joint 7 
        };




    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}

void leavePlaceThree(const robot_state::JointModelGroup* joint_model_group,
                     moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.793602,		 // Joint 1 
        +0.293711,		 // Joint 2 
        +2.425398,		 // Joint 3 
        -1.710999,		 // Joint 4 
        +0.358704,		 // Joint 5 
        +2.326216,		 // Joint 6 
        +0.096573		 // Joint 7 
        };



    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceSeven(const robot_state::JointModelGroup* joint_model_group,
                    moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.629001,		 // Joint 1 
        +0.332123,		 // Joint 2 
        +2.875330,		 // Joint 3 
        -2.677662,		 // Joint 4 
        -0.396351,		 // Joint 5 
        +3.259462,		 // Joint 6 
        +1.011538		 // Joint 7 
        };





    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.644874,		 // Joint 1 
        +0.044384,		 // Joint 2 
        +2.876314,		 // Joint 3 
        -2.513478,		 // Joint 4 
        -0.377289,		 // Joint 5 
        +3.348445,		 // Joint 6 
        +1.079716		 // Joint 7 
        };




    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}

bool leavePlaceSeven(const robot_state::JointModelGroup* joint_model_group,
                     moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.629001,		 // Joint 1 
        +0.332123,		 // Joint 2 
        +2.875330,		 // Joint 3 
        -2.677662,		 // Joint 4 
        -0.396351,		 // Joint 5 
        +3.259462,		 // Joint 6 
        +1.011538		 // Joint 7 
        };




    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceEight(const robot_state::JointModelGroup* joint_model_group,
                    moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.405430,		 // Joint 1 
        +0.055462,		 // Joint 2 
        +2.618529,		 // Joint 3 
        -2.489355,		 // Joint 4 
        -1.104544,		 // Joint 5 
        +3.744352,		 // Joint 6 
        +1.480782		 // Joint 7 
        };




    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.432385,		 // Joint 1 
        -0.160770,		 // Joint 2 
        +2.681322,		 // Joint 3 
        -2.348030,		 // Joint 4 
        -0.984436,		 // Joint 5 
        +3.739133,		 // Joint 6 
        +1.481079		 // Joint 7 
        };




    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}

bool leavePlaceEight(const robot_state::JointModelGroup* joint_model_group,
                     moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.405430,		 // Joint 1 
        +0.055462,		 // Joint 2 
        +2.618529,		 // Joint 3 
        -2.489355,		 // Joint 4 
        -1.104544,		 // Joint 5 
        +3.744352,		 // Joint 6 
        +1.480782		 // Joint 7 
        };




    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceNine(const robot_state::JointModelGroup* joint_model_group,
                   moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.251620,		 // Joint 1 
        +0.139746,		 // Joint 2 
        +2.852862,		 // Joint 3 
        -2.300409,		 // Joint 4 
        +1.745586,		 // Joint 5 
        +2.658023,		 // Joint 6 
        -1.256127		 // Joint 7 
        };




    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.284026,		 // Joint 1 
        -0.101459,		 // Joint 2 
        +2.863415,		 // Joint 3 
        -2.128719,		 // Joint 4 
        +1.742942,		 // Joint 5 
        +2.609693,		 // Joint 6 
        -1.250185		 // Joint 7 
        };




    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}

void leavePlaceNine(const robot_state::JointModelGroup* joint_model_group,
                    moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.251620,		 // Joint 1 
        +0.139746,		 // Joint 2 
        +2.852862,		 // Joint 3 
        -2.300409,		 // Joint 4 
        +1.745586,		 // Joint 5 
        +2.658023,		 // Joint 6 
        -1.256127		 // Joint 7 
        };



    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceFour(const robot_state::JointModelGroup* joint_model_group,
                   moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.704027,		 // Joint 1 
        +0.584230,		 // Joint 2 
        +2.875369,		 // Joint 3 
        -2.515814,		 // Joint 4 
        -0.455071,		 // Joint 5 
        +2.950280,		 // Joint 6 
        +1.117243		 // Joint 7 
        };



    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.725847,		 // Joint 1 
        +0.306364,		 // Joint 2 
        +2.863083,		 // Joint 3 
        -2.341458,		 // Joint 4 
        -0.448853,		 // Joint 5 
        +2.911509,		 // Joint 6 
        +1.117706		 // Joint 7 
        };



    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}

void leavePlaceFour(const robot_state::JointModelGroup* joint_model_group,
                    moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.704027,		 // Joint 1 
        +0.584230,		 // Joint 2 
        +2.875369,		 // Joint 3 
        -2.515814,		 // Joint 4 
        -0.455071,		 // Joint 5 
        +2.950280,		 // Joint 6 
        +1.117243		 // Joint 7 
        };




    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceFive(const robot_state::JointModelGroup* joint_model_group,
                   moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.899778,		 // Joint 1 
        +0.622851,		 // Joint 2 
        +2.525775,		 // Joint 3 
        -2.343862,		 // Joint 4 
        -0.234449,		 // Joint 5 
        +2.633637,		 // Joint 6 
        +0.642411		 // Joint 7 
        };


    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +2.044198,		 // Joint 1 
        +0.296671,		 // Joint 2 
        +2.394162,		 // Joint 3 
        -2.164684,		 // Joint 4 
        +0.020413,		 // Joint 5 
        +2.641030,		 // Joint 6 
        +0.481424		 // Joint 7 
        };





    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}

void leavePlaceFive(const robot_state::JointModelGroup* joint_model_group,
                    moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +1.899778,		 // Joint 1 
        +0.622851,		 // Joint 2 
        +2.525775,		 // Joint 3 
        -2.343862,		 // Joint 4 
        -0.234449,		 // Joint 5 
        +2.633637,		 // Joint 6 
        +0.642411		 // Joint 7 
        };



    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceSix(const robot_state::JointModelGroup* joint_model_group,
                  moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +2.439258,		 // Joint 1 
        +0.950777,		 // Joint 2 
        +1.958296,		 // Joint 3 
        -2.145058,		 // Joint 4 
        -0.788928,		 // Joint 5 
        +2.610972,		 // Joint 6 
        +0.723250		 // Joint 7 
        };



    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +2.660545,		 // Joint 1 
        +0.911755,		 // Joint 2 
        +1.782103,		 // Joint 3 
        -2.025104,		 // Joint 4 
        -0.883853,		 // Joint 5 
        +2.615542,		 // Joint 6 
        +0.767611		 // Joint 7 
        };




    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}

void leavePlaceSix(const robot_state::JointModelGroup* joint_model_group,
                   moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){

    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +2.439258,		 // Joint 1 
        +0.950777,		 // Joint 2 
        +1.958296,		 // Joint 3 
        -2.145058,		 // Joint 4 
        -0.788928,		 // Joint 5 
        +2.610972,		 // Joint 6 
        +0.723250		 // Joint 7 
        };




    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findRightSpot(const robot_state::JointModelGroup* joint_model_group,
                   moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, int place, moveit::core::RobotStatePtr current_state){


// Hier die Abfrage
// wenn 1 - 3: oberes Regal
//    auf Position des jeweiligen Plates fahren (davor schweben)
//    auf Ablageposition des Plates fahren und Gripper öffnen
// wenn 4 - 6: keine Änderung
//    auf Position des jeweiligen Plates fahren (davor schweben)
//    auf Ablageposition des Plates fahren und Gripper öffnen
// wenn 7 - 9: unteres Regal fahren
//    auf Position des jeweiligen Plates fahren (davor schweben)
//    auf Ablageposition des Plates fahren und Gripper öffnenbool notCorrectPosition = true;


    if (place < 4) {
// Hier Position für erstes Fach
        std::vector<double> joint_group_positions;
        current_state = move_group->getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        joint_group_positions =
        {
        +1.567893,		 // Joint 1 
        +0.962621,		 // Joint 2 
        +2.739953,		 // Joint 3 
        -2.295525,		 // Joint 4 
        +0.182554,		 // Joint 5 
        +2.547771,		 // Joint 6 
        +0.307719		 // Joint 7 
        };





        moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);


        if(place == 1) {
            findPlaceOne(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
        else if(place == 2) {
            // Hier Position für zweiten Platz
            findPlaceTwo(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
        else{
            // Hier Position für dritten Platz
            findPlaceThree(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
    }
    else if (place > 6) {
// Hier Position für drittes Fach
        std::vector<double> joint_group_positions;
        current_state = move_group->getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        joint_group_positions =
            {
            +1.472702,		 // Joint 1 
            +1.067248,		 // Joint 2 
            +2.641509,		 // Joint 3 
            -3.036880,		 // Joint 4 
            +0.054043,		 // Joint 5 
            +2.977073,		 // Joint 6 
            +0.195525		 // Joint 7 
            };





        moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

        if(place == 7) {
            findPlaceSeven(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);

        }
        else if(place == 8) {
            findPlaceEight(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);

        }
        else{
            findPlaceNine(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);

        }
    }
    else {

        if(place == 4) {
            findPlaceFour(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
        else if(place == 5) {
            findPlaceFive(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
        else{
            findPlaceSix(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool leaveRightSpot(const robot_state::JointModelGroup* joint_model_group,
                    moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, int place, moveit::core::RobotStatePtr current_state){

    if (place < 4) {
        if(place == 1) {
            leavePlaceOne(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
        else if(place == 2) {
            leavePlaceTwo(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
        else{
            leavePlaceThree(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
    }
    else if (place > 6) {
        if(place == 7) {
            leavePlaceSeven(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
        else if(place == 8) {
            leavePlaceEight(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
        else{
            leavePlaceNine(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
    }
    else {

        if(place == 4) {
            leavePlaceFour(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
        else if(place == 5) {
            leavePlaceFive(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
        else{
            leavePlaceSix(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveFromStorage(const robot_state::JointModelGroup* joint_model_group,
                     moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, int place, moveit::core::RobotStatePtr current_state){

    /*
    if (place < 4) {
// Hier Position für erstes Fach
        std::vector<double> joint_group_positions;
        current_state = move_group->getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        joint_group_positions =
            {
            -1.274421,		 // Joint 1 
            -1.484182,		 // Joint 2 
            -0.509653,		 // Joint 3 
            -2.539459,		 // Joint 4 
            -0.653390,		 // Joint 5 
            +2.489574,		 // Joint 6 
            +0.893266		 // Joint 7 
            };



        moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    }
    else if (place > 6) {
// Hier Position für drittes Fach
        std::vector<double> joint_group_positions;
        current_state = move_group->getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        joint_group_positions =
            {
            -1.499604,		 // Joint 1 
            -1.694284,		 // Joint 2 
            -0.715597,		 // Joint 3 
            -2.957918,		 // Joint 4 
            -0.779736,		 // Joint 5 
            +2.568088,		 // Joint 6 
            +0.867862		 // Joint 7 
            };


        moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    }
    */

// In Schwebeposition zurück fahren
    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +2.031532,		 // Joint 1 
        +1.054223,		 // Joint 2 
        +2.168870,		 // Joint 3 
        -2.671940,		 // Joint 4 
        -0.792631,		 // Joint 5 
        +3.065037,		 // Joint 6 
        +0.802607		 // Joint 7 
        };





    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}

void openGripper(actionlib::SimpleActionClient<franka_gripper::StopAction> *acs,
                 actionlib::SimpleActionClient<franka_gripper::MoveAction> *acm,
                 franka_gripper::StopGoal goalS, franka_gripper::MoveGoal goalM){
    acs->sendGoal(goalS);
    goalM.width = 0.08;
    goalM.speed = 0.1;
    acm->sendGoal(goalM);
}

void closeGripper(actionlib::SimpleActionClient<franka_gripper::GraspAction> *acg, franka_gripper::GraspGoal goalG){
    goalG.width = 0.04;
    goalG.speed = 0.1;
    goalG.force = 60;
    goalG.epsilon.inner = 0.05;
    goalG.epsilon.outer = 0.05;
    acg->sendGoal(goalG);
}

void homeGripper(actionlib::SimpleActionClient<franka_gripper::HomingAction> *ach, franka_gripper::HomingGoal goal){
    ach->sendGoal(goal);
    sleep(3);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ShowToCamera(const robot_state::JointModelGroup* joint_model_group,
                  moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){
    // show object to camera
    std::vector<double> joint_group_positions;

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
        {
        +0.861129,		 // Joint 1 
        +0.835717,		 // Joint 2 
        +0.199601,		 // Joint 3 
        -0.577428,		 // Joint 4 
        -0.169529,		 // Joint 5 
        +3.432604,		 // Joint 6 
        +0.994774		 // Joint 7 
        };




    sleep(1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool getBlockFromPrinterToOutput(const robot_state::JointModelGroup* joint_model_group,
                                 moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools,
                                 float speed, Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state,
                                 actionlib::SimpleActionClient<franka_gripper::GraspAction> *acg, actionlib::SimpleActionClient<franka_gripper::StopAction> *acs,
                                 actionlib::SimpleActionClient<franka_gripper::MoveAction> *acm, franka_gripper::GraspGoal goalG,
                                 franka_gripper::StopGoal goalS, franka_gripper::MoveGoal goalM){


    moveToPrinter(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    closeGripper(acg, goalG);
    sleep(0.5);

    moveFromPrinter(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    ShowToCamera(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    moveToOutput(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    openGripper(acs, acm, goalS, goalM);
    sleep(0.5);

    moveFromOutput(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    moveToInitialPosition(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool getBlockFromPrinterToStorage(const robot_state::JointModelGroup* joint_model_group,
                                  moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools,
                                  float speed,   Eigen::Affine3d text_pose, int place, moveit::core::RobotStatePtr current_state,
                                  actionlib::SimpleActionClient<franka_gripper::GraspAction> *acg, actionlib::SimpleActionClient<franka_gripper::StopAction> *acs,
                                  actionlib::SimpleActionClient<franka_gripper::MoveAction> *acm, franka_gripper::GraspGoal goalG,
                                  franka_gripper::StopGoal goalS, franka_gripper::MoveGoal goalM){

    moveToPrinter(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    closeGripper(acg, goalG);
    sleep(0.5);

    moveFromPrinter(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    ShowToCamera(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    moveToStorage(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    findRightSpot(joint_model_group, move_group, visual_tools, speed, text_pose, place, current_state);
    openGripper(acs, acm, goalS, goalM);
    sleep(0.5);

    leaveRightSpot(joint_model_group, move_group, visual_tools, speed, text_pose, place, current_state);
    moveFromStorage(joint_model_group, move_group, visual_tools, speed, text_pose, place, current_state);

    moveToInitialPosition(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool getBlockFromStorageToOutput(const robot_state::JointModelGroup* joint_model_group,
                                 moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,
                                 Eigen::Affine3d text_pose, int place, moveit::core::RobotStatePtr current_state,
                                 actionlib::SimpleActionClient<franka_gripper::GraspAction> *acg, actionlib::SimpleActionClient<franka_gripper::StopAction> *acs,
                                 actionlib::SimpleActionClient<franka_gripper::MoveAction> *acm, franka_gripper::GraspGoal goalG,
                                 franka_gripper::StopGoal goalS, franka_gripper::MoveGoal goalM){

    moveToStorage(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    findRightSpot(joint_model_group, move_group, visual_tools, speed, text_pose, place, current_state);
    closeGripper(acg, goalG);
    sleep(0.5);

    leaveRightSpot(joint_model_group, move_group, visual_tools, speed, text_pose, place, current_state);
    moveFromStorage(joint_model_group, move_group, visual_tools, speed, text_pose, place, current_state);
    ShowToCamera(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    moveToOutput(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    openGripper(acs, acm, goalS, goalM);
    sleep(0.5);

    moveFromOutput(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    moveToInitialPosition(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);

    return true;
}

//////////////////////////////////////////////  M A I N  ///////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    ////////////////// INIT //////////////////
    // ROS INIT
    ros::init(argc, argv, "Stretching");
    ros::NodeHandle node_handle;
    ros::Subscriber sub = node_handle.subscribe("ros_opcua_order", 1000, chatterCallback);
    ros::Publisher pub = node_handle.advertise<std_msgs::String>("ros_opcua_response", 1000);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // PANDA INIT
    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    const robot_state::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Z_FirstTry Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    double speed = 0.4;
    moveit::core::RobotStatePtr current_state;

    actionlib::SimpleActionClient<franka_gripper::GraspAction> acg("franka_gripper/grasp", true);
    actionlib::SimpleActionClient<franka_gripper::StopAction> acs("franka_gripper/stop", true);
    actionlib::SimpleActionClient<franka_gripper::MoveAction> acm("franka_gripper/move", true);
    actionlib::SimpleActionClient<franka_gripper::HomingAction> ach("franka_gripper/homing", true);
    // actionlib::SimpleActionClient<franka_control::ErrorRecoveryAction> era("franka_control/errorRecovery", true);

    ROS_INFO("Waiting for action server to start.");
    //wait for the action server to start
    acg.waitForServer(); //will wait for infinite time
    acs.waitForServer();
    acm.waitForServer();
    ach.waitForServer();
    // era.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    franka_gripper::GraspGoal goalG;
    franka_gripper::StopGoal goalS;
    franka_gripper::MoveGoal goalM;
    franka_gripper::HomingGoal goalH;
    // franka_control::ErrorRecoveryActionGoal goalError;

    // openGripper(&acs, &acm, goalS, goalM);

    ///////////////// MAIN LOOP //////////////////
    try{

        while(ros::ok()) {

            ////////////////// RECEIVING ORDERS //////////////////
            if (global_order_movement.compare("PS") == 0 || global_order_movement.compare("SO") == 0) {

                std::cout << "Place: " << global_order_pos << std::endl;

                // check if place is supported (must be between 1 and 9)
                if (global_order_pos < 1 || global_order_pos > 9) {
                    // std::cout << "Place not supported!" << std::endl;
                    global_order_movement = "XX";
                    global_order_pos = 0;
                    continue;
                }
            } else if (global_order_movement.compare("XX") == 0) {
                continue;
            }

            // publish state
            global_response = "Moving";
            // This is the Response from the robot if it is moving or not, necessary for the opcua master to wait until "stopped"
            // so that it could run the conveyor belt
            global_ros_response.data = global_response;
            pub.publish(global_ros_response);

            // try{
                ////////////////// MOVEMENT OF ROBOT //////////////////
                if (global_order_movement.compare("PO") == 0) {
                    homeGripper(&ach, goalH);
                    getBlockFromPrinterToOutput(joint_model_group, &move_group, visual_tools, speed, text_pose,
                                                current_state, &acg, &acs, &acm, goalG, goalS, goalM);
                } else if (global_order_movement.compare("PS") == 0) {
                    homeGripper(&ach, goalH);
                    getBlockFromPrinterToStorage(joint_model_group, &move_group, visual_tools, speed, text_pose, global_order_pos,
                                                current_state, &acg, &acs, &acm, goalG, goalS, goalM);
                } else if (global_order_movement.compare("SO") == 0) {
                    homeGripper(&ach, goalH);
                    getBlockFromStorageToOutput(joint_model_group, &move_group, visual_tools, speed, text_pose, global_order_pos,
                                                current_state, &acg, &acs, &acm, goalG, goalS, goalM);
                }
            // }catch(const std::exception& e){
            //     era.sendGoal(goalError);
            // }

            // publish state
            global_response = "Stopped";
            // This is the Response from the robot if it is moving or not, necessary for the opcua master to wait until "stopped"
            // so that it could run the conveyor belt
            global_ros_response.data = global_response;
            pub.publish(global_ros_response);

            // Reset Position, otherwise robot would move in an endless loop
            global_order_pos = 0;

            sleep(2);
        }

    } catch(const std::exception& e){
        std::cout << "Catched Exception: " << e.what() << " - stopping program" << std::endl;
        ros::shutdown();
        return 0;
    }
}

