cmake_minimum_required(VERSION 3.0.2)
project(utils)



## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
)
find_package(gazebo REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs nav_msgs visualization_msgs tf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${GAZEBO_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_definitions(${GAZEBO_CXX_FLAGS})


add_executable(catmull_rom_optimizer_node
  src/centripetal_catmull_rom_optimizer_node.cpp
)
target_link_libraries(catmull_rom_optimizer_node ${catkin_LIBRARIES})

add_executable(test_start_goal_node
  src/test_start_goal_node.cpp
)

target_link_libraries(test_start_goal_node
  ${catkin_LIBRARIES}
)
add_executable(test_edt_map_node
  src/test_edt_map_node.cpp
)

target_link_libraries(test_edt_map_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/los_detect.py
  scripts/waypoint_publisher.py
  scripts/test.py
  scripts/uav_navigator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
