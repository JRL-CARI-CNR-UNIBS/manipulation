cmake_minimum_required(VERSION 3.10.0)
project(manipulation_utils)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(catkin REQUIRED COMPONENTS
  manipulation_msgs
  moveit_planning_helper
  object_loader_msgs
  roscpp
  rosdyn_core
  rosparam_utilities
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES manipulation_utils
  CATKIN_DEPENDS manipulation_msgs moveit_planning_helper object_loader_msgs roscpp roscpp rosdyn_core rosparam_utilities std_srvs
  DEPENDS 
)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
            src/location.cpp
            src/go_to_location.cpp
            src/pick_objects.cpp
            src/place_objects.cpp
            src/manipulation_load_params_utils.cpp
            src/manipulation_utils.cpp 
            src/skill_base.cpp 
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(roscpp REQUIRED)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_test test/test.launch test/test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${roscpp_LIBRARIES} )

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES ${PROJECT_NAME}_test
    )
  endif()
endif()

set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-runtime/int,-whitespace/braces,+whitespace,-readability/function")
roslint_cpp()
