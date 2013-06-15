cmake_minimum_required(VERSION 2.8.3)
project(map_to_jpeg)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS sensor_msgs nav_msgs geometry_msgs tf image_transport cv_bridge)

include_directories(${EIGEN_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS})


catkin_package(
   INCLUDE_DIRS include ${EIGEN_INCLUDE_DIRS}
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp
   DEPENDS eigen OpenCV)


add_executable(map_to_image_node src/map_to_image_node.cpp)
target_link_libraries(map_to_image_node ${catkin_LIBRARIES})
target_link_libraries(map_to_image_node ${OpenCV_LIBRARIES})

add_executable(image_to_map_node src/image_to_map_node.cpp)
target_link_libraries(image_to_map_node ${catkin_LIBRARIES})
