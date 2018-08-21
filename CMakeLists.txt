cmake_minimum_required(VERSION 2.8.3)
project(gazebo_rangebearing_plugin)
find_package(catkin REQUIRED COMPONENTS gazebo_dev roscpp message_generation)
find_package(Eigen3 REQUIRED)


# Plugins require c++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories( #include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
)

## Declare a C++ library
add_library(gazebo_rangebearing_plugin
  src/RangeBearing.cc
  )

target_link_libraries(gazebo_rangebearing_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS gazebo_rangebearing_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
