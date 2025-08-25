cmake_minimum_required(VERSION 3.0.2)
project(teleopeletion_system)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  rospy
  std_msgs
  geometry_msgs
  collaboration_manipulation_message
)

#add_message_files( 
#  FILES
#  )

add_service_files(
  FILES
  NotifyPauseCompletion.srv
)

generate_messages( 
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  collaboration_manipulation_message
)
