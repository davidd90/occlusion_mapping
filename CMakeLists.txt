cmake_minimum_required(VERSION 2.8.3)
project(occlusion_mapping)

set(PACKAGE_DEPENDENCIES
    roscpp
    occlusion_mapping_msgs
    visualization_msgs
    sensor_msgs
    nodelet
    nav_msgs
    cv_bridge
    pcl_conversions
    pcl_ros
    tf
)


find_package(catkin REQUIRED COMPONENTS 
   ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)


find_package(OpenCV)

###########
## Build ##
###########
add_compile_options(-std=c++11)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
	${catkin_INCLUDE_DIRS}
)



## Declare a C++ library
add_library(${PROJECT_NAME}
        src/OcclusionMap.cpp
	src/GroundFilter.cpp
	src/Depthbuffer.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)


add_library(${PROJECT_NAME}_nodelet
	src/OcclusionMap.cpp
	src/GroundFilter.cpp
	src/Depthbuffer.cpp
	src/occlusion_map_nodelet.cpp
	src/groundFilter_nodelet.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_nodelet
	${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(occlusion_map_node src/occlusion_map_node.cpp)
target_link_libraries(occlusion_map_node ${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(groundFilter_node src/groundFilter_node.cpp)
target_link_libraries(groundFilter_node ${PROJECT_NAME} ${catkin_LIBRARIES})




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        occlusion_map_node
	groundFilter_node
	${PROJECT_NAME}_nodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(
	DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
