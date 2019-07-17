cmake_minimum_required(VERSION 2.8.12)

project(kamerider_control_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    geometry_msgs
    nav_msgs
    move_base_msgs
    std_msgs
    message_generation
)

add_message_files(
  FILES
    Mission.msg
    Result.msg
    MoveRobot.msg
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    nav_msgs
    move_base_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    nav_msgs
    move_base_msgs
    message_runtime
    std_msgs
)
