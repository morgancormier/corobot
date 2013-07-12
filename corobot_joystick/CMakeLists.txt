cmake_minimum_required(VERSION 2.8.3)
project(corobot_joystick)

find_package(catkin REQUIRED COMPONENTS corobot_msgs roscpp)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(joystick_control src/joystick_control.cpp)
target_link_libraries(joystick_control ${catkin_LIBRARIES})
add_dependencies(joystick_control corobot_msgs_gencpp)

