cmake_minimum_required(VERSION 2.8.3)
project(phidget_motor)

find_package(catkin REQUIRED)

catkin_package(
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS rospy)
