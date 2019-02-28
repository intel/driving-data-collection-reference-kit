cmake_minimum_required(VERSION 2.8.3)
project(cam_lib)

include(ExternalProject)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../../cmake/modules")
SET(CMAKE_CXX_FLAGS "-std=c++11")

ExternalProject_Add (
  APRILTAGS
  GIT_REPOSITORY "https://github.com/swatbotics/apriltags-cpp.git"
  GIT_TAG "master"
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/thirdParty/apriltags"
  UPDATE_COMMAND ""
  INSTALL_COMMAND "" )


find_package( OpenCV REQUIRED )
find_package(flycapture REQUIRED)

ExternalProject_Get_Property(APRILTAGS source_dir)
set(APRILTAGS_INCLUDE_DIR ${source_dir}/src)

# Library
ExternalProject_Get_Property(APRILTAGS binary_dir)
set(APRILTAGS_LIBRARY_PATH ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}apriltags.a)
set(APRILTAGS_LIBRARY apriltags)

add_library(${APRILTAGS_LIBRARY} UNKNOWN IMPORTED)
set_property(TARGET ${APRILTAGS_LIBRARY} PROPERTY IMPORTED_LOCATION
  ${APRILTAGS_LIBRARY_PATH} )
add_dependencies(${APRILTAGS_LIBRARY} APRILTAGS)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
  LIBRARIES cam_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM
  include/cam_lib/
  ${APRILTAGS_INCLUDE_DIR}
  ${LEVMAR_INCLUDE_DIR}
  ${FLYCAPTURE_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_library(cam_lib STATIC
  src/cam_base.cpp
  src/ptgrey.cpp
  src/usb_cam.cpp)

add_dependencies(cam_lib ${APRILTAGS_LIBRARY})


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

 install(TARGETS cam_lib 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})