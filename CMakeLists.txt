cmake_minimum_required(VERSION 2.8.3)
project(wc_gtsam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs rospy roscpp
)

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wc_gtsam
  CATKIN_DEPENDS
  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
#  src/Pose2SLAMExample_node.cpp
#  src/imufactorsexample_node.cpp
  src/VisualISAM2Example_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  gtsam
  ${catkin_LIBRARIES}
)

#add_executable(Pose2SLAMExample_node src/Pose2SLAMExample_node.cpp)
#target_link_libraries(Pose2SLAMExample_node
#  ${PROJECT_NAME}
#  gtsam
#  ${catkin_LIBRARIES}
#)

#add_executable(imufactorsexample_node src/imufactorsexample_node.cpp)
#target_link_libraries(imufactorsexample_node
#  ${PROJECT_NAME}
#  gtsam
#  ${catkin_LIBRARIES}
#)

add_executable(VisualISAM2Example_node src/VisualISAM2Example_node.cpp)
target_link_libraries(VisualISAM2Example_node
  ${PROJECT_NAME}
  gtsam
  ${catkin_LIBRARIES}
)
