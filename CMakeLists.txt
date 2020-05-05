cmake_minimum_required(VERSION 3.10.2)
project(geometry_cpp LANGUAGES CXX)

find_package(catkin REQUIRED 
  COMPONENTS
  cmake_modules # Needed to find Eigen3
)

############
## Eigen3 ##
############

find_package(Eigen3 REQUIRED)

# Remove warning caused by unset variable.
if (EIGEN3_INCLUDE_DIRS AND NOT Eigen3_INCLUDE_DIRS)
  set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
endif()

# Create interface target for Eigen
add_library(Eigen3_INTERFACE INTERFACE)
target_include_directories(Eigen3_INTERFACE
  INTERFACE
    ${EIGEN3_INCLUDE_DIRS}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    GeometryMath
    GeometryTrajectory
    GeometryLieGroup
#  CATKIN_DEPENDS ...
  DEPENDS Eigen3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

###################
## Library: math ##
###################

add_library(GeometryMath)

target_sources(GeometryMath
  PRIVATE
    src/math/types.cpp
)

target_include_directories(GeometryMath
  PUBLIC
    include
  PRIVATE
    include/geometry_cpp
)

target_link_libraries(GeometryMath
  PUBLIC
    Eigen3_INTERFACE
)

set_target_properties(GeometryMath
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CMAKE_CXX_EXTENSIONS OFF
)

#########################
## Library: trajectory ##
#########################

add_library(GeometryTrajectory)

target_sources(GeometryTrajectory
  PRIVATE
    src/trajectory/linear_trajectory.cpp
    # src/trajectory/angular_trajectory.cpp
)

target_include_directories(GeometryTrajectory
  PUBLIC
    include
  PRIVATE
    include/geometry_cpp
)

target_link_libraries(GeometryTrajectory
  PUBLIC
    GeometryMath
)

set_target_properties(GeometryTrajectory
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CMAKE_CXX_EXTENSIONS OFF
)

########################
## Library: lie_group ##
########################

add_library(GeometryLieGroup)

target_sources(GeometryLieGroup
  PRIVATE
    src/lie_group/utility.cpp
)

target_include_directories(GeometryLieGroup
  PUBLIC
    include
  PRIVATE
    include/geometry_cpp
)

target_link_libraries(GeometryLieGroup
  PUBLIC
    GeometryMath
)

set_target_properties(GeometryLieGroup
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CMAKE_CXX_EXTENSIONS OFF
)

######################
## Build properties ##
######################

# TODO?: Set flags directly on targets instead?
set(WARNING_FLAGS "-Wall -Wextra -Wpedantic -Wno-deprecated-copy")
list(APPEND CMAKE_CXX_FLAGS ${WARNING_FLAGS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark libraries for installation
install(TARGETS GeometryMath GeometryTrajectory GeometryLieGroup
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_localisation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
