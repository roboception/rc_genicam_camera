cmake_minimum_required(VERSION 3.5)

project(rc_genicam_camera)
include(cmake/project_version.cmake)

# - Configuration for code optimization -

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RELEASE CACHE STRING "Build type: DEBUG or RELEASE" FORCE)
endif ()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# checking for intel architecture and activating SSSE3 for optimization of
# rotating images

EXECUTE_PROCESS(COMMAND ${CMAKE_CXX_COMPILER} -dumpmachine COMMAND tr -d '\n' OUTPUT_VARIABLE CXX_MACHINE)
string(REGEX REPLACE "([a-zA-Z_0-9]+).*" "\\1" ARCHITECTURE ${CXX_MACHINE})

if ("${ARCHITECTURE}" STREQUAL "x86_64")
  set(USE_SSSE3 1 CACHE BOOL "Compile for SSEE3 (x86) using -mssse3")
else ()
  set(USE_SSSE3 0 CACHE BOOL "Compile for SSEE3 (x86) using -mssse3")
endif ()

if (USE_SSSE3)
  add_definitions(-mssse3)
endif ()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rc_genicam_camera_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(RC_GENICAM_API REQUIRED)

include_directories(
  include
  ${RC_GENICAM_API_INCLUDE_DIRS}
)

ament_export_include_directories("include/${PROJECT_NAME}")

add_executable(${PROJECT_NAME}_node
  src/genicam_camera_node_main.cc
  src/publishers/camera_info_publisher.cc
  src/publishers/image_publisher.cc
  src/camerainfolist.cc
  src/genicam_camera_node.cc
  src/imagelist.cc
  src/timestamp_corrector.cc
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  image_transport
  rc_genicam_camera_interfaces
)

target_link_libraries(${PROJECT_NAME}_node
  rc_genicam_api::rc_genicam_api
)

# ############
# # Install ##
# ############

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# ## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
# rc_genicam_camera_nodelet.xml
# README.md
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# #install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# ######################################
# ## Define information for packaging ##
# ######################################
# # optionally specify dependencies of the debian package here (comma separated!)
# set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF) # to prevent automatic addition of libs that are ORed in dependencies
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "rc-genicam-api (>= 2.0.3) | ros-$ENV{ROS_DISTRO}-rc-genicam-api (>= 2.0.3), ros-$ENV{ROS_DISTRO}-image-pipeline")

# include(cmake/package_debian.cmake)

ament_package()
