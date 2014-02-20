cmake_minimum_required(VERSION 2.8.3)
project(corobot_phidget_ik)

find_package(catkin REQUIRED COMPONENTS roscpp corobot_msgs sensor_msgs std_msgs corobot_diagnostics std_srvs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp corobot_diagnostics)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(corobot_phidget src/corobot_phidget.cpp src/orientation.cpp)
target_link_libraries(corobot_phidget phidget21)
target_link_libraries(corobot_phidget ${catkin_LIBRARIES})
add_dependencies(corobot_phidget corobot_msgs_gencpp)
