cmake_minimum_required(VERSION 3.5.1)
project(mir100_navigation)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  mir100_localization
  mir100_gazebo
  base_local_planner
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslaunch_add_file_check(launch)