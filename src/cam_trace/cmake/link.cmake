target_link_libraries(test_node
test_lib
${OpenCV_LIBS}
${catkin_LIBRARIES}
)

target_link_libraries(cam_node
cam_lib
ltproc_lib
${OpenCV_LIBS}
${catkin_LIBRARIES}
)

target_link_libraries(move_cam_node
move_cam_lib
dynamixel_lib
control_lib
${DYNAMIXEL_LIBRARIES}
${catkin_LIBRARIES}
)
