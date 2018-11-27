cmake_minimum_required(VERSION 2.8.3)
project(datainfile)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs message_generation sensor_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
  DIRECTORY msg
  FILES
  DataFile.msg
  ImageDataFile.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES datainfile
  CATKIN_DEPENDS roscpp message_runtime std_msgs
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${datainfile_INCLUDE_DIRS}
)

add_library(datainfile src/datafile.cpp)

add_dependencies(datainfile ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
