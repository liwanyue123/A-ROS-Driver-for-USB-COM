cmake_minimum_required(VERSION 3.0.2)
project(com_pkg)
add_compile_options(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(LIBCONFIG_LINK_DIR "/home/orin/catkin_ws/src/slamcar/third-party/controlcan/") #usb2can 添加1

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES com_pkg
 CATKIN_DEPENDS roscpp rospy serial std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${LIBCONFIG_LINK_DIR} #usb2can 添加2
)

 
add_library(
  util_lib
  src/Utilities/Period/PeriodicTask.cpp
  src/ComManager.cpp
)

#com ros程序
add_executable(textCom_main src/textCom_main.cpp) 
target_link_libraries(textCom_main   util_lib  ${catkin_LIBRARIES}  )#controlcan 是usb2can 添加4 因为目标动态库叫libcontrolcan.so

