cmake_minimum_required(VERSION 2.8.3)
project(corobot_pantilt)

find_package(catkin REQUIRED COMPONENTS corobot_msgs sensor_msgs std_msgs rosconsole joy diagnostic_updater corobot_diagnostics)

catkin_package(
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp sensor_msgs corobot_msgs rosconsole std_msgs joy corobot_diagnostics)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(corobot_pantilt src/pantilt.cpp src/dynctrl.c src/utils.c src/v4l2uvc.c)
target_link_libraries(corobot_pantilt ${catkin_LIBRARIES})
add_dependencies(corobot_pantilt corobot_msgs_gencpp)
