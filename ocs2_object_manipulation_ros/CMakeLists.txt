cmake_minimum_required(VERSION 3.0.2)
project(ocs2_object_manipulation_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_object_manipulation
  ocs2_robotic_assets
  gazebo_dev
  rosbag
)

find_package(catkin REQUIRED COMPONENTS
  roslib
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Mpc node
add_executable(object_mpc
  src/ObjectMpcNode.cpp
)
add_dependencies(object_mpc
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(object_mpc
  ${catkin_LIBRARIES}
)

# Mrt node
add_executable(object_mrt
  src/ObjectMrtNode.cpp
  src/ObjectDummyVisualization.cpp
)
add_dependencies(object_mrt
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(object_mrt
  ${catkin_LIBRARIES}
)

# Apply Wrench node
add_executable(apply_wrench_node
  src/ApplyWrenchNode.cpp
)
add_dependencies(apply_wrench_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(apply_wrench_node
  ${catkin_LIBRARIES}
)

# Target Pose node
add_executable(object_manipulation_target
  src/TargetPoseCommand.cpp
)
add_dependencies(object_manipulation_target
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(object_manipulation_target
  ${catkin_LIBRARIES}
)

# Bagfile node
add_executable(bagfile_recorder
  src/bagfile_recorder.cpp
)
add_dependencies(bagfile_recorder
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(bagfile_recorder
  ${catkin_LIBRARIES}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Running clang tooling.")
  add_clang_tooling(
    TARGETS
        object_mpc
        object_dummy_test
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_EXCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
