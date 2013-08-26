cmake_minimum_required(VERSION 2.8.3)
project(corobot_gps)

find_package(catkin REQUIRED COMPONENTS corobot_msgs diagnostic_updater corobot_diagnostics)


catkin_package(
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp diagnostic_updater corobot_diagnostics)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(corobot_gps src/client.cpp)
target_link_libraries(corobot_gps ${catkin_LIBRARIES})
target_link_libraries(corobot_gps gps)
add_dependencies(corobot_gps corobot_msgs_gencpp)

