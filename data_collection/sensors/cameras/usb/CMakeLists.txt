cmake_minimum_required(VERSION 2.8.3)
project(usb_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation image_transport roscpp std_msgs std_srvs sensor_msgs camera_info_manager cv_bridge datainfile)

#find_package(CUDA REQUIRED)
find_package(Boost 1.54  REQUIRED COMPONENTS system thread)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -O2 -fopenmp -mavx2")

###################################################
## Declare things to be passed to other projects ##
###################################################



###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
  ${datainfile_INCLUDE_DIRS}
)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs datainfile
  INCLUDE_DIRS include ${datainfile_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
)

# Build the USB camera library
add_library(${PROJECT_NAME} src/usb_cam.cpp)
target_link_libraries(${PROJECT_NAME}
    yaml-cpp
    ${avcodec_LIBRARIES}
    ${swscale_LIBRARIES}
    ${catkin_LIBRARIES}
    ${datainfile_LIBRARIES}
    )
add_dependencies(usb_cam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Builde the USB camera nodelet
add_library(${PROJECT_NAME}_nodelet nodes/usb_cam_nodelet.cpp nodes/usb_cam_wrapper.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet
    ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
  ${datainfile_LIBRARIES}
)

## Declare USB camera cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp nodes/usb_cam_wrapper.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
  ${datainfile_LIBRARIES}
)
add_dependencies(usb_cam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_nodelet
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
  FILES_MATCHING PATTERN "*.yaml"
)

install(FILES nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

