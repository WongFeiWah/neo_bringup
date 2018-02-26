cmake_minimum_required(VERSION 2.8.3)
project(neo_bringup)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
)

catkin_package()

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(neo_bringup_node src/robot.cpp src/serial.cpp src/timeout.cpp)
target_link_libraries(neo_bringup_node
  ${catkin_LIBRARIES}
  pthread
)
