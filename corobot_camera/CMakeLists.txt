cmake_minimum_required(VERSION 2.8.3)
project(corobot_camera)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp sensor_msgs image_transport camera_calibration_parsers corobot_diagnostics)

generate_dynamic_reconfigure_options(cfg/parameters.cfg)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS dynamic_reconfigure roscpp sensor_msgs image_transport camera_calibration_parsers corobot_diagnostics
   DEPENDS OpenCV SDL)

include_directories(include ${catkin_INCLUDE_DIRS} cfg/cpp ${corobot_diagnostics_INCLUDE_DIRS}) 

include_directories(${SDL_INCLUDE_DIRS})
include_directories(include)
link_directories(${SDL_LIBRARY_DIRS})
add_definitions("-D_FILE_OFFSET_BITS=64")

#add dynamic reconfigure api

# add the uvc_cam library
add_library(uvc_cam src/uvc_cam/uvc_cam.cpp)
target_link_libraries(uvc_cam avcodec swscale)

add_executable(view src/view/view.cpp)
target_link_libraries(view uvc_cam)
target_link_libraries(view SDL)
target_link_libraries(view ${catkin_LIBRARIES})

add_executable(enumerate src/enumerate/enumerate.cpp)
target_link_libraries(enumerate uvc_cam)

add_executable(dump src/dump/dump.cpp src/dump/avilib.c)
target_link_libraries(dump uvc_cam)
target_link_libraries(dump ${catkin_LIBRARIES})

add_executable(dynamic_sender src/dynamic_sender.cpp)
add_dependencies(dynamic_sender corobot_camera_gencfg)
target_link_libraries(dynamic_sender ${catkin_LIBRARIES}) 
target_link_libraries(dynamic_sender uvc_cam)
target_link_libraries(dynamic_sender ${OpenCV_LIBRARIES})

add_executable(record_theora src/record_theora.cpp)
target_link_libraries(record_theora theora theoraenc theoradec ogg uvc_cam)
target_link_libraries(record_theora ${catkin_LIBRARIES})

add_executable(dump_send src/dump/dump_send.cpp src/dump/avilib.c)
target_link_libraries(dump_send uvc_cam)
target_link_libraries(dump_send ${catkin_LIBRARIES})
