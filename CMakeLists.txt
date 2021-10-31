cmake_minimum_required(VERSION 3.0.2)
project(diff_drive)

## Find catkin macros and libraries
find_package(
  catkin
  REQUIRED
  COMPONENTS
  urdf
  xacro
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS
  urdf
  xacro
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  nodes/follow_rect.py
  nodes/flip_over.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 )

#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
