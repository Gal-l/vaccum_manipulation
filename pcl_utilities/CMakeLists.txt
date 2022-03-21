cmake_minimum_required(VERSION 2.8.3)
project(pcl_utilities)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages



find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  pickommerce_msgs
#yaml-cpp
)

find_package(PCL)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${YAML_CPP_INCLUDEDIR})
link_directories(${YAML_CPP_LIBRARIES})
add_definitions(${YAML_CPP_DEFINITIONS})

add_executable(save_to_pcd src/save_to_pcd.cpp)
add_dependencies(save_to_pcd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(save_to_pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dummy_camera src/dummy_camera.cpp)
add_dependencies(dummy_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dummy_camera ${catkin_LIBRARIES} ${PCL_LIBRARIES})



