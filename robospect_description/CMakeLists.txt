cmake_minimum_required(VERSION 2.8.3)
project(robospect_description)

find_package(catkin REQUIRED COMPONENTS
  message_runtime
  std_msgs
  std_srvs
  tf
  robotnik_sensors
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robospect_description
#  CATKIN_DEPENDS message_runtime std_msgs std_srvs tf
#  DEPENDS system_lib
)
