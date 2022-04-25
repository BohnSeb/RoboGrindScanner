cmake_minimum_required(VERSION 3.0.2)
project(artec)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED)



################################################
## Declare ROS messages, services and actions ##
################################################


find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 lib
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(testscan src/artec.cpp)


target_link_libraries(testscan
  ${catkin_LIBRARIES} 
  ${PROJECT_SOURCE_DIR}/lib/artec-sdk-algorithms.lib
  ${PROJECT_SOURCE_DIR}/lib/artec-sdk-base.lib
  ${PROJECT_SOURCE_DIR}/lib/artec-sdk-capturing.lib
  ${PROJECT_SOURCE_DIR}/lib/artec-sdk-project.lib
  ${PROJECT_SOURCE_DIR}/lib/artec-sdk-scanning.lib
)
add_custom_command(TARGET testscan POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PROJECT_SOURCE_DIR}/dll"
        $<TARGET_FILE_DIR:testscan>)