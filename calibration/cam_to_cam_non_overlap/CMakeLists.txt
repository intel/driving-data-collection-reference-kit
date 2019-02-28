cmake_minimum_required(VERSION 2.8.3)
project(ext_calib_cam_cam_non_overlap)

include(ExternalProject)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../../cmake/modules")
SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package( OpenCV REQUIRED )
find_package(flycapture REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp cam_lib)

# This driver uses Boost threads
find_package(Boost REQUIRED COMPONENTS thread)

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
INCLUDE_DIRS ${cam_lib_INCLUDE_DIRS}
CATKIN_DEPENDS roscpp  cam_lib
LIBRARIES ${PROJECT_NAME} ${cam_lib_LIBS}
)

link_directories(${catkin_LIBRARY_DIRS} ${cam_lib_LIBRARY_DIRS})

include_directories(SYSTEM
  ${APRILTAGS_INCLUDE_DIR}
  ${LEVMAR_INCLUDE_DIR}
  ${FLYCAPTURE_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cam_lib_INCLUDE_DIRS})

add_executable (save_tag_details
  save_tag_details.cpp
  )

target_link_libraries(save_tag_details
   ${OpenCV_LIBS}
   ${FLYCAPTURE_LIBRARIES}
   ${APRILTAGS_LIBRARY}
   ${cam_lib_LIBRARIES}

   )

   add_dependencies(save_tag_details ${cam_lib_EXPORTED_TARGETS})

add_executable (save_cam_details
  save_cam_details.cpp
  )

target_link_libraries(save_cam_details
   ${OpenCV_LIBS}
   ${FLYCAPTURE_LIBRARIES}
   ${APRILTAGS_LIBRARY}
   ${cam_lib_LIBRARIES}
   )

add_dependencies(save_cam_details ${cam_lib_EXPORTED_TARGETS})

add_executable (sheet_detail_creator
  sheet_detail_creator.cpp
  )

target_link_libraries(sheet_detail_creator
   ${OpenCV_LIBS}
   ${FLYCAPTURE_LIBRARIES}
   ${APRILTAGS_LIBRARY}
  ${cam_lib_LIBRARIES} 
   )

   add_dependencies(sheet_detail_creator ${cam_lib_EXPORTED_TARGETS})


add_executable (find_tag_rel find_tag_rel.cpp)
target_link_libraries(find_tag_rel ${OpenCV_LIBS})

add_executable (find_cam_rel find_cam_rel.cpp)
target_link_libraries(find_cam_rel ${OpenCV_LIBS})


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
