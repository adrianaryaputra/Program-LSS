cmake_minimum_required(VERSION 3.0.2)
project(lss_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  CATKIN_DEPENDS rospy 
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  src/boat_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)