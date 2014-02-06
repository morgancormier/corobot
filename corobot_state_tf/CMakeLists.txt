cmake_minimum_required(VERSION 2.8.3)
project(corobot_state_tf)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp tf)

generate_dynamic_reconfigure_options(cfg/parameters.cfg)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp dynamic_reconfigure)

include_directories(include ${catkin_INCLUDE_DIRS} cfg/cpp)

add_executable(corobot_state_tf src/state_tf.cpp src/utility.cpp)
add_dependencies(corobot_state_tf corobot_state_tf_gencfg)
target_link_libraries(corobot_state_tf ${catkin_LIBRARIES}) 

