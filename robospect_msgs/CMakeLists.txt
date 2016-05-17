cmake_minimum_required(VERSION 2.8.3)
project(robospect_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy
  std_msgs
  genmsg
  message_generation
  actionlib
  actionlib_msgs
  nav_msgs
  geometry_msgs
)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
	FILES
	State.msg
	ControlState.msg
	MissionCommand.msg
	MissionPoint.msg
	MissionState.msg
	PlatformCommand.msg
	PlatformState.msg
	PlatformResponse.msg
	PadStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY srv
	FILES
	DetectCrack.srv
	GetStereo.srv
	ScanProfile.srv
	SetControlMode.srv
	PlatformCommandSrv.srv	
	MissionCommandSrv.srv	
)

#add_action_files(DIRECTORY action 
#	FILES 
#	Task.action)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs # actionlib_msgs Or other packages containing msgs
   geometry_msgs
)

catkin_package()


