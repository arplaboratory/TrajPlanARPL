cmake_minimum_required(VERSION 2.8.2)
project(traj_gen)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)


set(CMAKE_VERBOSE_MAKEFILE on)
set(CMAKE_CXX_FLAGS "-fPIC")
# add_definitions(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
MESSAGE( STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  nav_msgs
  nlopt
  tf
)

find_package(ooqp REQUIRED ooqpbase ooqpdense ooqpgensparse ooqpmehrotra ooqpbound ooqpgendense
ooqpgondzio ooqpsparse)
find_package(Eigen3 REQUIRED)
find_package(catkin_simple REQUIRED)
find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)


include_directories(include ${NLOPT_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${OOQP_INCLUDE_DIRS})

# System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
INCLUDE_DIRS include  ${EIGEN3_INCLUDE_DIR} ${OOQP_INCLUDE_DIR}  ${NLOPT_INCLUDE_DIRS}
LIBRARIES traj_gen
CATKIN_DEPENDS roscpp 
)

#add_dependencies(ooqpei)

add_library(traj_gen 
src/traj_src/Waypoint.cpp 
src/traj_src/TrajBase.cpp 
src/traj_src/QPpolyTraj.cpp
#src/traj_src/Nlopt_traj.cpp
src/traj_src/FOV.cpp
src/ros_utils_src/polynomial.cc
src/ros_utils_src/EigenQP.cpp
src/ros_utils_src/quaternion.cpp
src/ooqp_interface/OoqpEigenInterface.cpp
src/ooqp_interface/QuadraticProblemFormulation.cpp
)

add_dependencies(traj_gen ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} )
  


target_link_libraries(traj_gen
   ${catkin_LIBRARIES}   
     nlopt blas ma27 gfortran
       ${OOQP_LIBRARIES} ${OOQP_LIBRARY}  
       ${NLOPT_LIBRARIES} ${NLOPT_LIBRARY}    
 )
   
   
install(TARGETS traj_gen
		EXPORT traj_gen_targets
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

