cmake_minimum_required(VERSION 3.0.2)
project(pickommerce_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  message_runtime
)


## Generate messages in the 'msg' folder
   add_message_files(
   FILES
   ClusterObjectList.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   ClusterList.srv
   Grippers_switcher.srv
   GraspPoint.srv
   PoseGoal.srv
   PakingData.srv
   empty.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker pickommerce_msgs_generate_messages_cpp)


