
set(SRC
    genicam_camera_nodelet.cc
    publishers/camera_info_publisher.cc
    publishers/image_publisher.cc)

# build and install the nodelet

link_directories(${RC_GENICAM_API_LIBRARY_DIRS})

add_library(rc_genicam_camera_nodelet ${SRC})

target_link_libraries(rc_genicam_camera_nodelet ${catkin_LIBRARIES})
target_link_libraries(rc_genicam_camera_nodelet ${RC_GENICAM_API_LIBRARIES})

add_executable(rc_genicam_camera genicam_camera_node.cc)
target_link_libraries(rc_genicam_camera ${catkin_LIBRARIES})

install(TARGETS rc_genicam_camera rc_genicam_camera_nodelet
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#add_dependencies(rc_genicam_camera_nodelet rc_genicam_camera_gencfg)
