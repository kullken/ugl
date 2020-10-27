cmake_minimum_required(VERSION 3.10.2)
project(ugl VERSION 0.0.1)

find_package(Eigen3 3.3 REQUIRED)

add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

add_library(project_warnings INTERFACE)
target_compile_options(project_warnings
  INTERFACE
    -Wall -Wextra -Wpedantic
    -Wshadow
    -Wnon-virtual-dtor
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wconversion
    -Wsign-conversion
    -Wnull-dereference
    -Wdouble-promotion
    -Wmisleading-indentation
)

include(cmake/StaticAnalyzers.cmake)
include(cmake/Sanitizers.cmake)

enable_sanitizers(project_options)

###################
## Library: math ##
###################

add_library(math SHARED)
add_library(ugl::math ALIAS math)

target_sources(math
  PRIVATE
    src/math/matrix.cpp
    src/math/quaternion.cpp
    src/math/slerp.cpp
)

target_include_directories(math
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(math
  PUBLIC
    Eigen3::Eigen
  PRIVATE
    project_options
    project_warnings
)

########################
## Library: lie_group ##
########################

add_library(lie_group SHARED)
add_library(ugl::lie_group ALIAS lie_group)

target_sources(lie_group
  PRIVATE
    src/lie_group/euclidean.cpp
    src/lie_group/rotation.cpp
    src/lie_group/rotation2d.cpp
    src/lie_group/pose.cpp
    src/lie_group/extended_pose.cpp
    src/lie_group/jacobian_helper.cpp
)

target_include_directories(lie_group
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/lie_group>
)

target_link_libraries(lie_group
  PUBLIC
    math
  PRIVATE
    project_options
    project_warnings
)

#########################
## Library: trajectory ##
#########################

add_library(trajectory SHARED)
add_library(ugl::trajectory ALIAS trajectory)

target_sources(trajectory
  PRIVATE
    src/trajectory/trajectory.cpp
    src/trajectory/bezier.cpp
    src/trajectory/bezier_sequence.cpp
    src/trajectory/slerp_sequence.cpp
    src/trajectory/circle_arc.cpp
)

target_include_directories(trajectory
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(trajectory
  PUBLIC
    math
    lie_group
  PRIVATE
    project_options
    project_warnings
)

#####################
## Library: random ##
#####################

add_library(random SHARED)
add_library(ugl::random ALIAS random)

target_sources(random
  PRIVATE
    src/random/random_engine.cpp
    src/random/seed.cpp
    src/random/normal_distribution.cpp
    src/random/uniform_distribution.cpp
)

target_include_directories(random
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/random>
)

target_link_libraries(random
  PUBLIC
    ugl::math
  PRIVATE
    project_options
    project_warnings
    Eigen3::Eigen
)

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS math trajectory lie_group random
  EXPORT ${PROJECT_NAME}-export
  ARCHIVE DESTINATION lib/ugl
  LIBRARY DESTINATION lib/ugl
  RUNTIME DESTINATION bin
)

install(EXPORT ${PROJECT_NAME}-export
  FILE uglTargets.cmake
  NAMESPACE ugl::
  DESTINATION lib/cmake/ugl
)

install(FILES "cmake/uglConfig.cmake"
  DESTINATION lib/cmake/ugl
)
