target_link_libraries(twist_to_pwm_node
twist_to_pwm_lib
${catkin_LIBRARIES}
)

target_link_libraries(rocker_node
${catkin_LIBRARIES}
)

target_link_libraries(pub_pwm_node
${catkin_LIBRARIES}
)

target_link_libraries(twist_to_action_node
twist_to_action_lib
    ${catkin_LIBRARIES}
)

target_link_libraries(qtr_to_reward_node
${catkin_LIBRARIES}
)

target_link_libraries(action_to_twist_node
action_to_twist_lib
    ${catkin_LIBRARIES}
)

target_link_libraries(cheat_node
${catkin_LIBRARIES}
)
