cmake_minimum_required(VERSION 2.8.3)
project(j3_moveit_config)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  serial
  actionlib
)

add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
	FILES
	joint_msg.msg
)

 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
#CATKIN_DEPENDS roscpp std_msgs system_lib
#  DEPENDS system_lib
    CATKIN_DEPENDS 
    message_runtime
    serial

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(jointcontroller src/RobotTrajectoryFollower.cpp)
add_executable(jointserial src/jointserial.cpp)

add_dependencies(jointcontroller j3_moveit_config_generate_messages_cpp)
add_dependencies(jointserial j3_moveit_config_generate_messages_cpp)

target_link_libraries(jointserial ${catkin_LIBRARIES})
target_link_libraries(jointcontroller ${catkin_LIBRARIES})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




