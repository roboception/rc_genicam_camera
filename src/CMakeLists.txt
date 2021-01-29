
set(SRC
    genicam_camera_nodelet.cc
    publishers/camera_info_publisher.cc
    publishers/image_publisher.cc
    timestamp_corrector.cc
    imagelist.cc
    camerainfolist.cc)

# build and install the nodelet

add_library(rc_genicam_camera_nodelet ${SRC})

target_link_libraries(rc_genicam_camera_nodelet
  PUBLIC
    ${catkin_LIBRARIES}
  PRIVATE
    rc_genicam_api::rc_genicam_api
)

add_executable(rc_genicam_camera genicam_camera_node.cc)
target_link_libraries(rc_genicam_camera ${catkin_LIBRARIES})

install(TARGETS rc_genicam_camera rc_genicam_camera_nodelet
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#add_dependencies(rc_genicam_camera_nodelet rc_genicam_camera_gencfg)
