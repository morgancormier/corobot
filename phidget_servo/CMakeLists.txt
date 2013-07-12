cmake_minimum_required(VERSION 2.8.3)
project(phidget_servo)

find_package(catkin REQUIRED COMPONENTS corobot_msgs roscpp)

catkin_package(
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp)

add_executable(phidget_servo src/PhidgetServo.cpp)
target_link_libraries(phidget_servo phidget21)
target_link_libraries(phidget_servo ${catkin_LIBRARIES})
add_dependencies(phidget_servo corobot_msgs_gencpp) 
