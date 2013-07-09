cmake_minimum_required(VERSION 2.8.3)
project(corobot_arm)

find_package(catkin REQUIRED COMPONENTS corobot_msgs rosconsole std_msgs xmlrpcpp roscpp)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rosconsole std_msgs xmlrpcpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(corobot_arm src/corobot_arm.cpp)
target_link_libraries(corobot_arm ${catkin_LIBRARIES})
add_dependencies(corobot_arm corobot_msgs_gencpp)
