cmake_minimum_required(VERSION 3.13) 
project(offboard_test)
# set(CMAKE_CXX_STANDARD 11)            # 设定C++标准，如果是C，则是CMAKE_C_STANDARD
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_BUILD_TYPE "Debug")	
find_package(Eigen3 REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  mavros_msgs
)

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBEVDEV REQUIRED libevdev)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES keyboard_ctrl
#  CATKIN_DEPENDS libevdev roscpp std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS mavros_msgs
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(/usr/include/libevdev-1.0/)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
)
file(GLOB SRC_FILES "src/*cpp")
add_executable(${PROJECT_NAME} ${SRC_FILES})

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBEVDEV REQUIRED libevdev)

target_include_directories(offboard_test PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${LIBEVDEV_INCLUDE_DIRS} 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/offboard_test>)

target_link_libraries(offboard_test PRIVATE
  ${LIBEVDEV_LIBRARIES}
  ${catkin_LIBRARIES})


install(TARGETS offboard_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)