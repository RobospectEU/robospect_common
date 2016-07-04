cmake_minimum_required(VERSION 2.8.3)
project(robospect_pad)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  robotnik_msgs
  roscpp
  sensor_msgs
  robospect_msgs
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(robospect_pad_node src/robospect_pad.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(robospect_pad_node robospect_msgs_generate_messages_cpp)
add_dependencies(robospect_pad_node robotnik_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robospect_pad_node
   ${catkin_LIBRARIES}
)
