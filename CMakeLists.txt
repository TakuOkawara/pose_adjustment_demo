cmake_minimum_required(VERSION 3.8)
project(demo_g2o_gtsam)

# Set the default C++ standard to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

add_compile_options(-O2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Iridescence REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Iridescence_INCLUDE_DIR})
include_directories(include)

link_directories(${GTSAM_LIBRARY_DIR})

# Pose Graph Optimization 2D, this code is based on GTSAM tutorial
add_executable(small_pose_adjustment_2d src/small_pose_adjustment_2d.cpp src/demo_iridescence.cpp src/customfactor.cpp)
target_link_libraries(small_pose_adjustment_2d gtsam 
  ${Iridescence_LIBRARIES} ${EIGEN3_LIBRARIES})
target_include_directories(small_pose_adjustment_2d PUBLIC
  ${Iridescence_INCLUDE_DIRS})

# Pose Graph Optimization 2D using manhattan dataset
add_executable(pose_adjustment_2d src/pose_adjustment_2d.cpp src/demo_iridescence.cpp src/customfactor.cpp)
target_link_libraries(pose_adjustment_2d gtsam 
  ${Iridescence_LIBRARIES} ${EIGEN3_LIBRARIES})
target_include_directories(pose_adjustment_2d PUBLIC
  ${Iridescence_INCLUDE_DIRS})

# Pose Graph Optimization 3D using sphere dataset
add_executable(pose_adjustment_3d src/pose_adjustment_3d.cpp src/demo_iridescence.cpp src/customfactor.cpp)
target_link_libraries(pose_adjustment_3d gtsam 
  ${Iridescence_LIBRARIES} ${EIGEN3_LIBRARIES})
target_include_directories(pose_adjustment_3d PUBLIC
  ${Iridescence_INCLUDE_DIRS})

# Pose Graph Optimization 3D using sphere dataset with custom between factor
# This custom between factor is the same as BetweenFactor<Pose3>
add_executable(pose_adjustment_3d_with_customfactor src/pose_adjustment_3d_with_customfactor.cpp src/demo_iridescence.cpp src/customfactor.cpp)
target_link_libraries(pose_adjustment_3d_with_customfactor gtsam 
  ${Iridescence_LIBRARIES} ${EIGEN3_LIBRARIES})
target_include_directories(pose_adjustment_3d_with_customfactor PUBLIC
  ${Iridescence_INCLUDE_DIRS})






