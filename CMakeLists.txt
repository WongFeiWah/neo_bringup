cmake_minimum_required(VERSION 2.8.3)
project(neo_bringup)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
)

catkin_package()

file(GLOB INCLUDE_PATH_LIST RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/neo_bringup/*.h)
file(GLOB INCLUDE_HPP_PATH_LIST RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/neo_bringup/*.hpp)
file(GLOB NEO_BRINGUP_CPP_PATH_LIST RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/neo_bringup/*.cpp)
file(GLOB COM_CPP_PATH_LIST RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/com/*.cpp)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(neo_bringup_node src/robot.cpp ${INCLUDE_PATH_LIST} ${INCLUDE_HPP_PATH_LIST} ${NEO_BRINGUP_CPP_PATH_LIST} ${COM_CPP_PATH_LIST})
target_link_libraries(neo_bringup_node
  ${catkin_LIBRARIES}
  pthread
)

add_executable(neo_sonar_node  src/sonar_node.cpp ${INCLUDE_PATH_LIST} ${INCLUDE_HPP_PATH_LIST} ${COM_CPP_PATH_LIST})
target_link_libraries(neo_sonar_node
        ${catkin_LIBRARIES}
        pthread
)
