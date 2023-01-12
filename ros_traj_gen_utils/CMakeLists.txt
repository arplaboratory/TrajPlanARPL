cmake_minimum_required(VERSION 3.0.2)
project(ros_traj_gen_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "-Wl,--no-undefined")
set(CMAKE_VERBOSE_MAKEFILE on)
set(CMAKE_CXX_FLAGS "-fPIC")
# add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  nav_msgs
  traj_gen
  message_generation
  visualization_msgs
  quadrotor_msgs
  trackers_msgs
  std_srvs
)

include_directories(include  ${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  Polynomial.msg
  StdTraj.msg
  cuboid_map.msg
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
)



catkin_package(
INCLUDE_DIRS include 
LIBRARIES ros_traj_gen_utils
CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs  traj_gen message_runtime  quadrotor_msgs trackers_msgs std_srvs
)


add_library(ros_traj_gen_utils
      src/pub_sub/ros_waypoint_utils.cpp
      src/pub_sub/ros_traj_utils.cpp 
      src/pub_sub/apriltag_utils.cpp #Move to Apriltag package later
      src/pub_sub/odom_utils.cpp 
      src/pub_sub/poscmd_publisher.cpp	
      src/cuboid_map/rect_node.cpp	
      src/cuboid_map/ros_cuboid_utils.cpp	
      src/ros_replanner_utils.cpp	
)

add_dependencies(ros_traj_gen_utils  ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} )

target_link_libraries(ros_traj_gen_utils ${catkin_LIBRARIES}  )

#	${pluginlib_LIBRARY}  ${actionlib_LIBRARY}  ${trackers_manager_LIBRARY}  ${trackers_msgs_LIBRARY} 
find_package(quadrotor_msgs)
add_executable(traj_exe src/traj_manager.cpp)

add_dependencies(traj_exe  ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} )

target_link_libraries(traj_exe ${catkin_LIBRARIES}    blas ma27 gfortran 
    ${OOQP_LIBRARIES} ros_traj_gen_utils)
