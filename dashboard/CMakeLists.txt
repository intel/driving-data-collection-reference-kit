cmake_minimum_required(VERSION 2.8.3)

project(linux_dashboard)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS roscpp bondcpp std_msgs rosnode )
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

catkin_package(
  CATKIN_DEPENDS roscpp bondcpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(/usr/lib/x86_64-linux-gnu/)


add_executable(dashboard src/dashboard.cpp)


target_link_libraries( dashboard -lmosquitto -lmosquittopp -ljson
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} ) 


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
