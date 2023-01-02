cmake_minimum_required(VERSION 3.0.2)
project(tello_ros)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(tello src/tello.cpp)
add_dependencies(tello ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tello ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(tello_ros_node nodes/tello_ros_node.cpp)
add_dependencies(tello_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tello_ros_node ${catkin_LIBRARIES} tello)
