cmake_minimum_required(VERSION 3.10.2)
project(ugl VERSION 0.0.1)

find_package(Eigen3 3.3 REQUIRED)

add_library(warnings INTERFACE)
target_compile_options(warnings
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
    -Wduplicated-cond
    -Wduplicated-branches
    -Wuseless-cast
)

###################
## Library: math ##
###################

add_library(math)
add_library(ugl::math ALIAS math)

target_sources(math
  PRIVATE
    src/math/matrix.cpp
    src/math/quaternion.cpp
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
    warnings
)

set_target_properties(math
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CMAKE_CXX_EXTENSIONS OFF
)

########################
## Library: lie_group ##
########################

add_library(lie_group)
add_library(ugl::lie_group ALIAS lie_group)

target_sources(lie_group
  PRIVATE
    src/lie_group/utility.cpp
    src/lie_group/mappings.cpp
)

target_include_directories(lie_group
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(lie_group
  PUBLIC
    math
  PRIVATE
    Eigen3::Eigen
    warnings
)

set_target_properties(lie_group
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CMAKE_CXX_EXTENSIONS OFF
)

#########################
## Library: trajectory ##
#########################

add_library(trajectory)
add_library(ugl::trajectory ALIAS trajectory)

target_sources(trajectory
  PRIVATE
    src/trajectory/trajectory.cpp
    src/trajectory/bezier_sequence.cpp
    src/trajectory/slerp_sequence.cpp
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
    warnings
)

set_target_properties(trajectory
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CMAKE_CXX_EXTENSIONS OFF
)

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS math trajectory lie_group 
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
