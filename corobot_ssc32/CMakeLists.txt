cmake_minimum_required(VERSION 2.8.3)
project(corobot_ssc32)

find_package(catkin REQUIRED COMPONENTS roscpp corobot_msgs diagnostic_updater corobot_diagnostics)

catkin_package(
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp corobot_diagnostics)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ssc32_motor src/motor.cpp src/ssc32.cpp)
target_link_libraries(ssc32_motor ${catkin_LIBRARIES})
add_dependencies(ssc32_motor corobot_msgs_gencpp)
