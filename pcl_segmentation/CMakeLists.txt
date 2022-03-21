cmake_minimum_required(VERSION 2.8.3)
project(pcl_segmentation)

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

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)



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

add_executable(euclidean_segmentation src/euclidean_segmentation.cpp)
add_dependencies(euclidean_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(euclidean_segmentation pickommerce_msgs_generate_messages_cpp)
target_link_libraries(euclidean_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})


