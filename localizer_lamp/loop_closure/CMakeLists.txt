cmake_minimum_required(VERSION 2.8.3)
project(loop_closure)

#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Release)
#ENDIF()

SET(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTSAM REQUIRED)
if (NOT GTSAM_FOUND)
  message(FATAL_ERROR "This program requires the GTSAM library.")
endif(NOT GTSAM_FOUND)

FIND_PACKAGE(Boost REQUIRED
  date_time
  serialization
  thread
  filesystem
  system
  regex
  timer
  chrono
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  lamp_utils
  geometry_utils
  parameter_utils
  point_cloud_mapper
  pcl_ros
  pcl_conversions
  std_msgs
  pose_graph_msgs
  geometry_msgs
  silvus_msgs

)

find_package(OpenMP)
# if (OpenMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
# endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    lamp_utils
    geometry_utils
    parameter_utils
    point_cloud_mapper
    pcl_ros
    pcl_conversions
    std_msgs
    pose_graph_msgs
    geometry_msgs
    silvus_msgs
  DEPENDS
    Boost
)

find_package(Eigen3 REQUIRED)

set(KISS_MATCHER_SOURCE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../KISS-Matcher/cpp/kiss_matcher"
)
if (NOT EXISTS "${KISS_MATCHER_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "KISS-Matcher core was not found at ${KISS_MATCHER_SOURCE_DIR}. "
    "Clone it to /home/nlg/catkin1_ws/src/KISS-Matcher."
  )
endif()
set(USE_SYSTEM_TBB ON CACHE BOOL
  "Use Ubuntu 20 system TBB instead of KISS-Matcher's fetched oneTBB."
  FORCE
)
add_subdirectory(
  "${KISS_MATCHER_SOURCE_DIR}"
  "${CMAKE_CURRENT_BINARY_DIR}/kiss_matcher"
)
if (TARGET robin)
  set(ROBIN_STD_VECTOR_COMPAT_HEADER
    "${CMAKE_CURRENT_BINARY_DIR}/robin_std_vector_compat.hpp"
  )
  file(WRITE "${ROBIN_STD_VECTOR_COMPAT_HEADER}"
    "#pragma once\n#include <vector>\nusing std::vector;\n"
  )
  target_compile_options(robin PRIVATE
    -include "${ROBIN_STD_VECTOR_COMPAT_HEADER}"
  )
endif()

set(SOLID_SOURCE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../SOLiD/cpp"
)
if (NOT EXISTS "${SOLID_SOURCE_DIR}/include/solid_module.h")
  message(FATAL_ERROR
    "SOLiD was not found at ${SOLID_SOURCE_DIR}. "
    "Clone it to /home/nlg/catkin1_ws/src/SOLiD."
  )
endif()

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
if (COMPILER_SUPPORTS_CXX17)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
  message(FATAL_ERROR "${CMAKE_CXX_COMPILER} doesn't provide c++17 support.")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${SOLID_SOURCE_DIR}/include
)
link_directories(${catkin_LIBRARY_DIRS} ${GTSAM_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(${PROJECT_NAME} 
  src/LoopGeneration.cc
  src/LoopPrioritization.cc
  src/LoopComputation.cc
  src/ProximityLoopGeneration.cc
  src/GenericLoopPrioritization.cc
  src/ObservabilityLoopPrioritization.cc
  src/IcpLoopComputation.cc
  src/LoopCandidateQueue.cc
  src/TestUtils.cc
  src/RoundRobinLoopCandidateQueue.cc
  src/ObservabilityQueue.cc
  src/RssiLoopClosure.cc
  src/LoopClosureBase.cc
  src/LaserLoopClosure.cc
)

find_package(Eigen3 REQUIRED)
find_package( PCL 1.8 REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
  Eigen3::Eigen
  kiss_matcher::kiss_matcher_core
)

add_executable(loop_generation_node src/loop_generation_node.cc)
target_link_libraries(loop_generation_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

add_executable(loop_prioritization_node src/loop_prioritization_node.cc)
target_link_libraries(loop_prioritization_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

add_executable(loop_candidate_queue_node src/loop_candidate_queue_node.cc)
target_link_libraries(loop_candidate_queue_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

add_executable(loop_computation_node src/loop_computation_node.cc)
target_link_libraries(loop_computation_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

add_executable(rssi_loop_generation_node src/rssi_loop_generation_node.cc)
target_link_libraries(rssi_loop_generation_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

add_executable(solid_loop_condition_node
  src/solid_loop_condition_node.cc
  ${SOLID_SOURCE_DIR}/src/solid.cpp
)
target_link_libraries(solid_loop_condition_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
  Eigen3::Eigen
)

add_executable(laser_loop_closure_node src/laser_loop_closure_node.cc)
target_link_libraries(laser_loop_closure_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

add_executable(gen_loop_computation_test src/gen_loop_computation_test.cc)
target_link_libraries(gen_loop_computation_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

add_executable(eval_loop_computation_test src/eval_loop_computation_test.cc)
target_link_libraries(eval_loop_computation_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
find_package(rostest REQUIRED)
  # add_subdirectory(test)
  ## Add gtest based cpp test target and link libraries
  add_rostest_gtest(test_icp_computation test/test_icp_computation.test test/test_icp_computation.cpp)
  target_link_libraries(test_icp_computation  ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(test_loop_generation test/test_loop_generation.test test/test_loop_generation.cc)
  target_link_libraries(test_loop_generation ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(test_rssi_loop_generation test/test_rssi_loop_generation.test test/test_rssi_loop_generation.cc)
  target_link_libraries(test_rssi_loop_generation ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(test_loop_prioritization test/test_loop_prioritization.test test/test_loop_prioritization.cc)
  target_link_libraries(test_loop_prioritization ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(test_loop_computation test/test_loop_computation.test test/test_loop_computation.cc)
  target_link_libraries(test_loop_computation ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
