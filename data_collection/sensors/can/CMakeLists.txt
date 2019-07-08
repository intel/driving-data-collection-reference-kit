cmake_minimum_required(VERSION 2.8.3)
project(can_node)

set (CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)
find_package(Boost REQUIRED COMPONENTS thread)

## System dependencies are found with CMake's conventions
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

link_directories(${catkin_LIBRARY_DIRS})
link_directories(/usr/lib/)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


add_message_files(
   FILES
   can_frame.msg
   can_frame_array.msg
)

generate_messages(DEPENDENCIES std_msgs )



###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(can src/can_node.cpp  src/kvaser_can.cpp)
add_dependencies(can ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(can
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lrt  -lcanlib -L/usr/lib/ -fpermissive
   ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
