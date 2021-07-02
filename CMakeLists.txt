cmake_minimum_required(VERSION 3.0.2)
project(pid_example)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pid_example
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
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

## The macro catkin_package() specifies catkin-specific information, such as exported
## headers, exported libraries and dependencies on other packages. This function must
## be called before declaring any targets with add_library() or add_executable().
catkin_package(
  ## the directories containing the headers exported by your project are 
  ## specified using INCLUDE_DIRS
  INCLUDE_DIRS include
  ## if you project exports any library targets, specify them using LIBRARIES  
  # LIBRARIES example_ros_library
  CATKIN_DEPENDS roscpp std_msgs rospy
  ## dependencies on non-catkin packages, if any, are specified using DEPENDS
  # DEPENDS OpenCV
)

include_directories(
  include ${catkin_INCLUDE_DIRS}

)

## Define a variable for all source files
set(example_ros_cpp_node_SRCS
      src/pid_controller_base.cpp
      src/pid_controller_ros.cpp
      src/nodes/pid_controller_node.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
## Declare a C++ executable target
add_executable(pid_controller_node ${example_ros_cpp_node_SRCS})
## Specify libraries to link a library or executable target against
target_link_libraries(pid_controller_node
  ${catkin_LIBRARIES}
  ## If the executable target depends on a library, it needs to be linked against it here
  # auxiliary_library
)