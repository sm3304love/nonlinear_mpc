cmake_minimum_required(VERSION 3.0.2)
project(nonlinear_mpc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-DEIGEN_STACK_ALLOCATION_LIMIT=0 -g")

find_package(mpc++ CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hpp-fcl REQUIRED)

# find_package(OpenMP REQUIRED)
# if(OPENMP_FOUND)
#   message(STATUS "OPENMP FOUND")
#   set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  
#   set(OpenMP_LIBS gomp)
# endif()


find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES nonlinear_mpc
 CATKIN_DEPENDS roscpp std_msgs gazebo_msgs geometry_msgs sensor_msgs pluginlib
 DEPENDS system_lib # OpenMP
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${hpp-fcl_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME} src/nonlinear_mpc.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20 )
add_dependencies(${PROJECT_NAME} mpc++ ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} osqp)
target_link_libraries(${PROJECT_NAME} mpc++ pinocchio::pinocchio)#${OpenMP_LIBS}

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} pinocchio::pinocchio mpc++)


# add_executable(${PROJECT_NAME} src/nmpc_planner.cpp)
# target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${PROJECT_NAME} casadi pinocchio::pinocchio hpp-fcl::hpp-fcl)

