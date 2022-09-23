cmake_minimum_required(VERSION 3.0.2)
project(biped_wbms_core)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  geometry_msgs
  cpp_filters)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3)


generate_dynamic_reconfigure_options(
  cfg/BipedWbmsCore.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES biped_wbms_core
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS EIGEN
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(biped_wbms_core src/biped_wbms_core.cpp)
target_link_libraries(biped_wbms_core
  ${catkin_LIBRARIES}
  ${EIGEN_LIBRARIES}
  )

#############
## Install ##
#############

install(TARGETS biped_wbms_core
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
