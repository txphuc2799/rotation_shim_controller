cmake_minimum_required(VERSION 3.0.2)
project(rotation_shim_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  cmake_modules
  base_local_planner
  costmap_2d
  nav_core
  nav_msgs
  pluginlib
  geometry_msgs
  angles
  visualization_msgs
  dynamic_reconfigure
)

find_package(Boost REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/RotationShimControllerReconfigure.cfg
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    base_local_planner
    nav_msgs
    roscpp
    tf2
    tf2_ros
    pluginlib
    angles
    dynamic_reconfigure
)

###########
## Build ##
###########

# Specify header include paths
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/footprint_collision_checker.cpp
                            src/rotation_shim_controller.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

# Install targets
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install plugins
install(FILES rotation_shim_controller_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
)