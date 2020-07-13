cmake_minimum_required(VERSION 3.10.2)
project(ugl LANGUAGES CXX)

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
    ugl_math
    ugl_lie_group
    ugl_trajectory
#  CATKIN_DEPENDS ...
  DEPENDS Eigen3
)

add_library(warnings INTERFACE)
target_compile_options(warnings
  INTERFACE
    -Wall -Wextra -Wpedantic
)

###################
## Library: math ##
###################

add_library(ugl_math
  src/math/matrix.cpp
  src/math/quaternion.cpp
)

target_include_directories(ugl_math
  PUBLIC
    include
)

target_link_libraries(ugl_math
  PUBLIC
  PRIVATE
    warnings
)

set_target_properties(ugl_math
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CMAKE_CXX_EXTENSIONS OFF
)

########################
## Library: lie_group ##
########################

add_library(ugl_lie_group
  src/lie_group/utility.cpp
  src/lie_group/mappings.cpp
)

target_include_directories(ugl_lie_group
  PUBLIC
    include
)

target_link_libraries(ugl_lie_group
  PUBLIC
    ugl_math
  PRIVATE
    warnings
)

set_target_properties(ugl_lie_group
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CMAKE_CXX_EXTENSIONS OFF
)

#########################
## Library: trajectory ##
#########################

add_library(ugl_trajectory
  src/trajectory/trajectory.cpp
  src/trajectory/bezier_sequence.cpp
  src/trajectory/slerp_sequence.cpp
)

target_include_directories(ugl_trajectory
  PUBLIC
    include
)

target_link_libraries(ugl_trajectory
  PUBLIC
    ugl_math
    ugl_lie_group
  PRIVATE
    warnings
)

set_target_properties(ugl_trajectory
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CMAKE_CXX_EXTENSIONS OFF
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark libraries for installation
install(TARGETS ugl_math ugl_trajectory ugl_lie_group
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
