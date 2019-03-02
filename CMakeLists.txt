cmake_minimum_required(VERSION 2.8.3)
project(voice_recognition)

find_package(catkin REQUIRED COMPONENTS
	rospy
)

catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS
	message_runtime
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES # e.g. Message.msgs MoreMessages.msgs
# )

## Generate services in the 'srv' folder
# add_service_files(
#  FILES  # e.g. Floats.srv HeaderString.srv
#)

## Generate added messages and services with any dependencies
#generate_messages()

include_directories(
	INCLUDE
	${catkin_INCLUDE_DIRS}
)

catkin_package(
	CATKIN_DEPENDS
	message_runtime
)

# catkin_install_python(PROGRAMS bin/rostest.py
#	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
