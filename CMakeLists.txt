cmake_minimum_required(VERSION 2.8.3)
project(mcr_object_recognition)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    pluginlib
    nodelet
    roscpp
    sensor_msgs
    geometry_msgs
    mcr_perception_msgs
    mcr_scene_segmentation
)

find_package(PCL 1.7 REQUIRED)
find_package(Eigen REQUIRED)

catkin_python_setup()

catkin_package(
    INCLUDE_DIRS
      ros/include
    LIBRARIES
      object_recognition
    CATKIN_DEPENDS
      mcr_perception_msgs
      rospy
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


### LIBRARIES ####################################################
add_library(object_recognition
#  ros/src/object_recognition_coordinator.cpp
  ros/src/object_recognition_esf.cpp
)
add_dependencies(object_recognition
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(object_recognition
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

### INSTALLS

install(TARGETS object_recognition
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

