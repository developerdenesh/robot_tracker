cmake_minimum_required(VERSION 3.0.2)
project(robot_tracker)

# Compile as C++11, supported in ROS Noetic
add_compile_options(-std=c++11)

# Find catkin macros and libraries
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ddynamic_reconfigure

  # This is used to find package path
  roslib
)

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES cars
  CATKIN_DEPENDS roscpp rospy std_msgs ddynamic_reconfigure

  # DEPENDS system_lib
)

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(
  test-library
  src/robot_tracker.cpp
)

# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/robot_tracker_node.cpp src/robot_tracker.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# # Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test test/robot_tracker_tests.cpp)
  target_link_libraries(${PROJECT_NAME}-test test-library ${catkin_LIBRARIES} ${catkin_LIBRARIES})
endif()
