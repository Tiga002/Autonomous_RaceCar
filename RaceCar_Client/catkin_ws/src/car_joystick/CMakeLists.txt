cmake_minimum_required(VERSION 2.8.3)
project(car_joystick)

## Compile as C++11, supported in ROS Kinetic and newe
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES js_controller
  CATKIN_DEPENDS joy roscpp std_msgs
  DEPENDS system_lib
)

set(local_include_dir ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(local_src_dir ${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(car_joystick ${local_src_dir}/main.cpp ${local_src_dir}/car_joystick.cpp)
target_link_libraries(car_joystick ${catkin_LIBRARIES})
