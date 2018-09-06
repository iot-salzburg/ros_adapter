cmake_minimum_required(VERSION 3.0)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

include(FindProtobuf)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

include_directories(${PROTOBUF_INCLUDE_DIR})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  examples_common.cpp
)

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3 ${PROTOBUF_LIBRARY})

set(EXAMPLES
  cartesian_impedance_control
  echo_robot_state
  force_control
  generate_cartesian_pose_motion
  generate_cartesian_velocity_motion
  generate_consecutive_motions
  generate_elbow_motion
  generate_joint_position_motion
  generate_joint_velocity_motion
  grasp_object
  joint_impedance_control
  joint_point_to_point_motion
  motion_with_control
  print_joint_poses
  Position_Checking
  ColorfulIndustrialUsecase
  IndustrialDemoCase
  Kistenstapler
  MoveToContact
  PiTest
  PiValueCalculator
  srfg_echo_robot_state
  kick_off_event
  kick_off_event_x
  dtz_echo_robot_state
)

#set(EXAMPLESCC
#  dtz_robot_message
#)

#foreach(examplecc ${EXAMPLESCC})
#  add_executable(${examplecc} ${examplecc}.pb)
#endforeach

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} Franka::Franka examples_common Eigen3::Eigen3)
endforeach()

target_link_libraries(joint_impedance_control Threads::Threads)
target_link_libraries(motion_with_control Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
