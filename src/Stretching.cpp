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
            {-0.000207,            // Joint 1
             -0.785368,            // Joint 2
             -0.000246,            // Joint 3
             -2.356503,           // Joint 4
             +0.000946,           // Joint 5
             +1.570938,            // Joint 6
             +0.784972             // Joint 7
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
            {+0.599266,            // Joint 1
             +0.162699,            // Joint 2
             +0.724710,            // Joint 3
             -2.069251,           // Joint 4
             +0.599960,           // Joint 5
             +1.795413,            // Joint 6
             +1.741219             // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    // Position direkt beim Drucker
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {+0.521981,            // Joint 1
             +0.803022,            // Joint 2
             +0.837095,            // Joint 3
             -2.182017,           // Joint 4
             +0.250329,           // Joint 5
             +2.425177,            // Joint 6
             +1.805058             // Joint 7
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
            {+0.599266,            // Joint 1
             +0.162699,            // Joint 2
             +0.724710,            // Joint 3
             -2.069251,           // Joint 4
             +0.599960,           // Joint 5
             +1.795413,            // Joint 6
             +1.741219             // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-0.136200,            // Joint 1
             +0.503317,            // Joint 2
             +0.069321,            // Joint 3
             -0.904044,           // Joint 4
             -0.008270,           // Joint 5
             +2.089224,            // Joint 6
             +0.763456              // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToOutput(const robot_state::JointModelGroup* joint_model_group,
                  moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){
    // Erste Position
    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-1.675980,            // Joint 1
             -0.067966 ,            // Joint 2
             +0.801821,            // Joint 3
             -1.808331,           // Joint 4
             -0.241695,           // Joint 5
             +2.056401,            // Joint 6
             -1.637750              // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

// Zweite Position
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-1.521378,            // Joint 1
             -0.678895,            // Joint 2
             +0.988494,            // Joint 3
             -2.171208,           // Joint 4
             +0.300746,           // Joint 5
             +1.933613,            // Joint 6
             -1.462331              // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

// Position über dem Förderband
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-2.600428,            // Joint 1
             -0.694723,            // Joint 2
             +2.300770,            // Joint 3
             -2.332446,           // Joint 4
             -1.497476,           // Joint 5
             +2.798764,           // Joint 6
             +0.365014             // Joint 7
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
            {-1.521378,            // Joint 1
             -0.678895,            // Joint 2
             +0.988494,            // Joint 3
             -2.171208,           // Joint 4
             +0.300746,           // Joint 5
             +1.933613,            // Joint 6
             -1.462331              // Joint 7
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
            {-0.099726,            // Joint 1
             +1.237739,            // Joint 2
             -1.972287,            // Joint 3
             -2.136524,           // Joint 4
             +0.355537,           // Joint 5
             +2.225301,            // Joint 6
             -1.403869             // Joint 7
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
            {-0.516120,            // Joint 1
             +0.820823,            // Joint 2
             -1.837487,            // Joint 3
             -1.347663,           // Joint 4
             +0.079926,           // Joint 5
             +1.910257,            // Joint 6
             -1.434026             // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-0.802786,            // Joint 1
             +1.120201,            // Joint 2
             -1.879392,            // Joint 3
             -1.399256,           // Joint 4
             +0.268353,           // Joint 5
             +2.069475,            // Joint 6
             -1.462804             // Joint 7
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
            {-0.516120,            // Joint 1
             +0.820823,            // Joint 2
             -1.837487,            // Joint 3
             -1.347663,           // Joint 4
             +0.079926,           // Joint 5
             +1.910257,            // Joint 6
             -1.434026             // Joint 7
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
            {-0.256274,            // Joint 1
             +0.859337,            // Joint 2
             -2.170525,            // Joint 3
             -1.627315,           // Joint 4
             +0.280060,           // Joint 5
             +1.833001,            // Joint 6
             -1.646919             // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-0.484176,            // Joint 1
             +1.137741,            // Joint 2
             -2.052988,            // Joint 3
             -1.883259,           // Joint 4
             +0.233163,           // Joint 5
             +2.289737,            // Joint 6
             -1.573190             // Joint 7
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
            {-0.256274,            // Joint 1
             +0.859337,            // Joint 2
             -2.170525,            // Joint 3
             -1.627315,           // Joint 4
             +0.280060,           // Joint 5
             +1.833001,            // Joint 6
             -1.646919             // Joint 7
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
            {-0.000185,            // Joint 1
             +0.976473,            // Joint 2
             -2.466721,            // Joint 3
             -1.934626,           // Joint 4
             +0.272974,           // Joint 5
             +1.871381,            // Joint 6
             -1.827909             // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-0.216743,            // Joint 1
             +1.213158,            // Joint 2
             -2.272480,            // Joint 3
             -2.278846,           // Joint 4
             +0.234566,           // Joint 5
             +2.447569,            // Joint 6
             -1.752545            // Joint 7
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
            {-0.000185,            // Joint 1
             +0.976473,            // Joint 2
             -2.466721,            // Joint 3
             -1.934626,           // Joint 4
             +0.272974,           // Joint 5
             +1.871381,            // Joint 6
             -1.827909             // Joint 7
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
            {-0.428316,            // Joint 1
             +1.122033,            // Joint 2
             -1.603507,            // Joint 3
             -1.884648,           // Joint 4
             +0.362244,           // Joint 5
             +2.221434,            // Joint 6
             -1.200215             // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-0.675392 ,            // Joint 1
             +1.360953,            // Joint 2
             -1.536964,            // Joint 3
             -1.853778,           // Joint 4
             +0.475122,           // Joint 5
             +2.432133,            // Joint 6
             -1.265867             // Joint 7
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
            {-0.428316,            // Joint 1
             +1.122033,            // Joint 2
             -1.603507,            // Joint 3
             -1.884648,           // Joint 4
             +0.362244,           // Joint 5
             +2.221434,            // Joint 6
             -1.200215             // Joint 7
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
            {-0.145144,            // Joint 1
             +1.120037,            // Joint 2
             -1.758764,            // Joint 3
             -2.318438,           // Joint 4
             +0.466797,           // Joint 5
             +2.419395,            // Joint 6
             -1.436929             // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-0.405421,            // Joint 1
             +1.357507,            // Joint 2
             -1.555525,            // Joint 3
             -2.327296,           // Joint 4
             +0.475904,           // Joint 5
             +2.697504,            // Joint 6
             -1.321866             // Joint 7
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
            {-0.145144,            // Joint 1
             +1.120037,            // Joint 2
             -1.758764,            // Joint 3
             -2.318438,           // Joint 4
             +0.466797,           // Joint 5
             +2.419395,            // Joint 6
             -1.436929             // Joint 7
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
            {+0.222048,            // Joint 1
             +1.211415,            // Joint 2
             -2.023364,            // Joint 3
             -2.721475,           // Joint 4
             +0.524792,           // Joint 5
             +2.463421,            // Joint 6
             -1.656938             // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-0.216917,            // Joint 1
             +1.304119,            // Joint 2
             -1.574034,            // Joint 3
             -2.722192,           // Joint 4
             +0.475608,           // Joint 5
             +2.988826,            // Joint 6
             -1.371405             // Joint 7
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
            {+0.222048,            // Joint 1
             +1.211415,            // Joint 2
             -2.023364,            // Joint 3
             -2.721475,           // Joint 4
             +0.524792,           // Joint 5
             +2.463421,            // Joint 6
             -1.656938             // Joint 7
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
            {-0.427535,            // Joint 1
             +0.979701,            // Joint 2
             -1.802566,            // Joint 3
             -1.729782,           // Joint 4
             +0.396360,           // Joint 5
             +2.114495,            // Joint 6
             -1.510005             // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-0.709982,            // Joint 1
             +1.289735,            // Joint 2
             -1.735670,            // Joint 3
             -1.695347,           // Joint 4
             +0.319118,           // Joint 5
             +2.314825,            // Joint 6
             -1.317854             // Joint 7
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
            {-0.427535,            // Joint 1
             +0.979701,            // Joint 2
             -1.802566,            // Joint 3
             -1.729782,           // Joint 4
             +0.396360,           // Joint 5
             +2.114495,            // Joint 6
             -1.510005             // Joint 7
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
            {-0.064589,            // Joint 1
             +1.006084,            // Joint 2
             -2.079785,            // Joint 3
             -2.105988,           // Joint 4
             +0.302475,           // Joint 5
             +2.155265,            // Joint 6
             -1.555202             // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-0.419530,            // Joint 1
             +1.245596,            // Joint 2
             -1.843816,            // Joint 3
             -2.162774,           // Joint 4
             +0.389124,           // Joint 5
             +2.525661,            // Joint 6
             -1.537265             // Joint 7
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
            {-0.064589,            // Joint 1
             +1.006084,            // Joint 2
             -2.079785,            // Joint 3
             -2.105988,           // Joint 4
             +0.302475,           // Joint 5
             +2.155265,            // Joint 6
             -1.555202             // Joint 7
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
            {+0.045320,            // Joint 1
             +1.277773,            // Joint 2
             -2.265168,            // Joint 3
             -2.379450,           // Joint 4
             +0.353567,           // Joint 5
             +2.044280,            // Joint 6
             -1.691176             // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-0.159131,            // Joint 1
             +1.262382,            // Joint 2
             -2.027054,            // Joint 3
             -2.547534,           // Joint 4
             +0.387758,           // Joint 5
             +2.592526,            // Joint 6
             -1.653765             // Joint 7
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
            {+0.045320,            // Joint 1
             +1.277773,            // Joint 2
             -2.265168,            // Joint 3
             -2.379450,           // Joint 4
             +0.353567,           // Joint 5
             +2.044280,            // Joint 6
             -1.691176             // Joint 7
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
                {-0.202298,            // Joint 1
                 +1.118617,            // Joint 2
                 -2.215788,            // Joint 3
                 -1.792656,           // Joint 4
                 +0.262851,           // Joint 5
                 +1.940611,            // Joint 6
                 -1.581900             // Joint 7
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
                {-0.087449,            // Joint 1
                 +1.312051,            // Joint 2
                 -1.758231,            // Joint 3
                 -2.331742,           // Joint 4
                 +0.478561,           // Joint 5
                 +2.410676,            // Joint 6
                 -1.306896             // Joint 7
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

    if (place < 4) {
// Hier Position für erstes Fach
        std::vector<double> joint_group_positions;
        current_state = move_group->getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        joint_group_positions =
                {-0.202298,            // Joint 1
                 +1.118617,            // Joint 2
                 -2.215788,            // Joint 3
                 -1.792656,           // Joint 4
                 +0.262851,           // Joint 5
                 +1.940611,            // Joint 6
                 -1.581900             // Joint 7
                };

        moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    }
    else if (place > 6) {
// Hier Position für drittes Fach
        std::vector<double> joint_group_positions;
        current_state = move_group->getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        joint_group_positions =
                {-0.087449,            // Joint 1
                 +1.312051,            // Joint 2
                 -1.758231,            // Joint 3
                 -2.331742,           // Joint 4
                 +0.478561,           // Joint 5
                 +2.410676,            // Joint 6
                 -1.306896             // Joint 7
                };

        moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    }

// In Schwebeposition zurück fahren
    std::vector<double> joint_group_positions;
    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-0.099726,            // Joint 1
             +1.237739,            // Joint 2
             -1.972287,            // Joint 3
             -2.136524,           // Joint 4
             +0.355537,           // Joint 5
             +2.225301,            // Joint 6
             -1.403869             // Joint 7
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
    sleep(5);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ShowToCamera(const robot_state::JointModelGroup* joint_model_group,
                  moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, moveit::core::RobotStatePtr current_state){
    // show object to camera
    std::vector<double> joint_group_positions;

    current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions =
            {-0.136200,            // Joint 1
             +0.503317,            // Joint 2
             +0.069321,            // Joint 3
             -0.904044,           // Joint 4
             -0.008270,           // Joint 5
             +2.089224,            // Joint 6
             +0.763456              // Joint 7
            };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
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
    sleep(1);

    moveFromPrinter(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    ShowToCamera(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    moveToOutput(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    openGripper(acs, acm, goalS, goalM);
    sleep(1);

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
    sleep(1);

    moveFromPrinter(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    ShowToCamera(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    moveToStorage(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    findRightSpot(joint_model_group, move_group, visual_tools, speed, text_pose, place, current_state);
    openGripper(acs, acm, goalS, goalM);
    sleep(1);

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
    sleep(1);

    leaveRightSpot(joint_model_group, move_group, visual_tools, speed, text_pose, place, current_state);
    moveFromStorage(joint_model_group, move_group, visual_tools, speed, text_pose, place, current_state);
    ShowToCamera(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    moveToOutput(joint_model_group, move_group, visual_tools, speed, text_pose, current_state);
    openGripper(acs, acm, goalS, goalM);
    sleep(1);

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

