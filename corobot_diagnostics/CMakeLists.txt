cmake_minimum_required(VERSION 2.8.3)
project(corobot_diagnostics)
find_package(catkin REQUIRED COMPONENTS rosconsole std_msgs xmlrpcpp roscpp diagnostic_aggregator diagnostic_msgs pluginlib)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rosconsole std_msgs xmlrpcpp diagnostic_aggregator diagnostic_msgs pluginlib)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(corobot_analyzer src/corobot_analyzer.cpp)
target_link_libraries(corobot_analyzer ${catkin_LIBRARIES})

add_executable(corobot_diagnostics src/lcd.cpp src/diagnostic.cpp)
#target_link_libraries(corobot_diagnostics ${catkin_LIBRARIES} picoLCD)
target_link_libraries(corobot_diagnostics ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
