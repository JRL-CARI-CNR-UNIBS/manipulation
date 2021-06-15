cmake_minimum_required(VERSION 3.10.0)
project(manipulation_msgs)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
  control_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
)

add_message_files(
  FILES
  Box.msg
  Grasp.msg
  Location.msg
  Object.msg
  Slot.msg
  SlotsGroup.msg
  Configuration.msg
  )

add_service_files(
  FILES
  AddBoxes.srv
  AddObjects.srv
  AddLocations.srv
  AddSlots.srv
  AddSlotsGroup.srv
  JobExecution.srv
  ListOfObjects.srv
  ListOfLocations.srv
  GetLocationIkSolution.srv
  RemoveBoxes.srv
  RemoveLocations.srv
  RemoveObjects.srv
  RemoveObjectFromSlot.srv
  RemoveSlots.srv
  RemoveSlotsGroup.srv
  ResetBoxes.srv
  ResetSlots.srv
  )

add_action_files(
  FILES
  PickObjects.action
  PlaceObjects.action
  GoTo.action
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs actionlib_msgs control_msgs sensor_msgs
)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
