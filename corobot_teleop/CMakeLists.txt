cmake_minimum_required(VERSION 2.8.3)
project(corobot_teleop)


find_package(catkin REQUIRED COMPONENTS roscpp corobot_msgs)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtWebKit QtOpengl)
include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp qt-ros)



file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
qt4_automoc(${QT_SOURCES})

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

include_directories(include ${catkin_INCLUDE_DIRS})

##############################################################################
# Binaries
##############################################################################

add_executable(corobot_teleop ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(corobot_teleop ${QT_LIBRARIES})
target_link_libraries(corobot_teleop ${catkin_LIBRARIES})
add_dependencies(corobot_teleop corobot_msgs_gencpp)
