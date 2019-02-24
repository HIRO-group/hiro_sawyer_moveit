cmake_minimum_required(VERSION 2.8.3)
project(hiro_sawyer_moveit)

set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  moveit_ros_planning_interface
  moveit_visual_tools
  intera_core_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
)

include_directories(
  SYSTEM ${Eigen_INCLUDE_DIRS}
         ${catkin_INCLUDE_DIRS}
)

add_executable(hiro_sawyer_moveit src/hiro_sawyer_moveit.cpp)
target_link_libraries(hiro_sawyer_moveit ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
