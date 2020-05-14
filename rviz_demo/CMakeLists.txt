##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(rviz_demo)

add_compile_options(-std=c++11)
set(CMAKE_AUTOUIC ON)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp std_msgs sensor_msgs rviz message_generation)

#add_message_files(FILES Chamber.msg Heartbeat.msg Valve.msg)
#add_service_files(FILES Task.srv)
#generate_messages(DEPENDENCIES std_msgs sensor_msgs)

include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(CATKIN_DEPENDS message_runtime)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
# Add the appropriate components to the component list here
find_package(Qt5 REQUIRED Core Gui Widgets)
set(QT_LIBRARIES Qt5::Widgets)

##############################################################################
# Sections
##############################################################################

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(rviz_demo ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#add_dependencies(rviz_demo rviz_demo_gencpp)
target_link_libraries(rviz_demo ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS rviz_demo RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

