cmake_minimum_required(VERSION 2.8.3)
project(sonar_detector)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sonar_detector src/sonar_detector.cpp)
add_dependencies(sonar_detector sensor_msgs_generate_messages_cpp)
 target_link_libraries(sonar_detector
   ${catkin_LIBRARIES}
)

