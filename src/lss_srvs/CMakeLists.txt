cmake_minimum_required(VERSION 3.0.2)
project(lss_srvs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
## Generate services in the 'srv' folder
add_service_files(
  FILES
  StringService.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs 
   std_msgs   
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)