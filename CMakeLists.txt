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
)

find_package(Boost REQUIRED)

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
    visualization_msgs
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
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

# Install targets
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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