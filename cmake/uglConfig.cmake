include(CMakeFindDependencyMacro)
find_dependency(Eigen3 3.3 REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/uglTargets.cmake")