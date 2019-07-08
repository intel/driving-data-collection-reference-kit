cmake_minimum_required(VERSION 3.0.2)

project(ext_calib_transform_validation)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp 
)

add_executable (transform_validation transform_validation.cpp )
target_link_libraries(transform_validation ${OpenCV_LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
