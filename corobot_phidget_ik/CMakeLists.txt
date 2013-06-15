cmake_minimum_required(VERSION 2.8.3)
project(corobot_phidget_ik)

find_package(catkin REQUIRED COMPONENTS roscpp corobot_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(corobot_phidget src/corobot_phidget.cpp)
target_link_libraries(corobot_phidget phidget21)
target_link_libraries(corobot_phidget ${catkin_LIBRARIES})
