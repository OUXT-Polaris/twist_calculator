cmake_minimum_required(VERSION 2.8.3)
project(twist_calculator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  tf2_ros
  tf2_geometry_msgs
  quaternion_operation
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/ImuToTwist.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES twist_calculator
  CATKIN_DEPENDS geometry_msgs roscpp tf2_ros tf2_geometry_msgs quaternion_operation dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# calculate twist from IMU
add_executable(imu_to_twist_node src/imu_to_twist_node.cpp src/imu_to_twist.cpp)
target_link_libraries(imu_to_twist_node ${catkin_LIBRARIES})

# calculate twist from pose
add_executable(pose_to_twist_node src/pose_to_twist_node.cpp src/pose_to_twist.cpp)
target_link_libraries(pose_to_twist_node ${catkin_LIBRARIES})

# install executables
install(TARGETS imu_to_twist_node pose_to_twist_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install headers
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)