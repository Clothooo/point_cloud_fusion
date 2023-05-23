cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -w ")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  pcl_ros
  sensor_msgs
  std_msgs
  stereo_msgs
  tf
  tf_conversions
  dynamic_reconfigure
  cmake_modules
  visualization_msgs
  # geometry_msgs
  # tf2
  # tf2_ros
  # tf2_sensor_msgs
  message_generation  # by jun
  # robot_vision # by jun
)

find_package(TinyXML REQUIRED)
find_package(PCL REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   ClusterCentroids.msg
#   CentersGT.msg  # by jun
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs  # by jun
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  DEPENDS TinyXML
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime  # by jun
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
  ${PCL_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/point_cloud_fusion.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/point_cloud_fusion_node.cpp)
add_executable(pcl_fusion src/pcl_fusion.cpp)
add_executable(pcl_fusion_real src/pcl_fusion_real.cpp)
add_executable(pcl_fusion_color src/pcl_fusion_color.cpp)

# add_executable(box_selection src/box_selection.cpp)
# add_executable(getdata_keyhd src/getdata_keyhd.cpp)
# add_executable(total_count src/total_count.cpp)



## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pcl_fusion
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(pcl_fusion_real
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(pcl_fusion_color
  ${catkin_EXPORTED_TARGETS}
)
# add_dependencies(box_selection
#   ${catkin_EXPORTED_TARGETS}
# )

# add_dependencies(getdata_keyhd
#   ${catkin_EXPORTED_TARGETS}
# )

# add_dependencies(total_count
#   ${catkin_EXPORTED_TARGETS}
# )

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(pcl_fusion
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

target_link_libraries(pcl_fusion_real
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

target_link_libraries(pcl_fusion_color
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${TinyXML_LIBRARIES}
)
# target_link_libraries(box_selection
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${TinyXML_LIBRARIES}
# )

# target_link_libraries(getdata_keyhd
#   ${catkin_LIBRARIES}
#   ${TinyXML_LIBRARIES}
# )

# target_link_libraries(total_count
#   ${catkin_LIBRARIES}
#   ${TinyXML_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_point_cloud_fusion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
