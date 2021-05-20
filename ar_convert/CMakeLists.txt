cmake_minimum_required(VERSION 3.1.3)

project(ar_convert)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
message_generation
roscpp
sensor_msgs
std_msgs
roslib
geometry_msgs
nav_msgs
tf
        )

catkin_package(    #  INCLUDE_DIRS include
#  LIBRARIES drawing_tools
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
LIBRARIES
INCLUDE_DIRS
CATKIN_DEPENDS
AlvarMarkers
roscpp
sensor_msgs
std_msgs
)

add_executable(main
        src/main.cpp src/ar_convert.cpp
        )

target_include_directories(main SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_include_directories(main PRIVATE include)

target_link_libraries(main ${OpenCV_LIBS} ${catkin_LIBRARIES})

