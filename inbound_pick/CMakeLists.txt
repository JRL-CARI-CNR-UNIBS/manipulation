cmake_minimum_required(VERSION 3.10.0)
project(inbound_pick)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  moveit_planning_helper
  manipulation_msgs
  manipulation_utils
  roscpp
  rosdyn_core
  std_srvs
  rosparam_utilities
  object_loader_msgs
  tf_conversions
  eigen_conversions
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES inbound_pick
  CATKIN_DEPENDS  moveit_planning_helper 
                  manipulation_msgs 
                  manipulation_utils 
                  roscpp 
                  rosdyn_core 
                  std_srvs 
                  rosparam_utilities 
                  object_loader_msgs 
                  tf_conversions
                  eigen_conversions
  DEPENDS
)
include_directories( ${catkin_INCLUDE_DIRS} )

add_executable(       ${PROJECT_NAME}_server src/inbound_pick_server_node.cpp)
add_dependencies(     ${PROJECT_NAME}_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_server ${catkin_LIBRARIES} )

add_executable(       ${PROJECT_NAME}_client src/inbound_pick_client_node.cpp)
add_dependencies(     ${PROJECT_NAME}_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_client ${catkin_LIBRARIES} )

add_executable(       ${PROJECT_NAME}_loader src/inbound_pick_loader_node.cpp)
add_dependencies(     ${PROJECT_NAME}_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_loader ${catkin_LIBRARIES} )

