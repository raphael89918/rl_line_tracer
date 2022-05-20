target_link_libraries(offline_collect_lib
conio
planner_lib
rl_handler_lib
${catkin_LIBRARIES}
)

target_link_libraries(online_training_lib
conio
planner_lib
rl_handler_lib
${catkin_LIBRARIES}
)

target_link_libraries(demo_lib
conio
planner_lib
rl_handler_lib
${catkin_LIBRARIES}
)

target_link_libraries(online_training_node
conio
online_training_lib
)

target_link_libraries(offline_collect_node
conio
offline_collect_lib
)

target_link_libraries(demo_node
conio
demo_lib
)

target_link_libraries(test_model_node
online_training_lib
)

target_link_libraries(test_rand_generator_node
${catkin_LIBRARIES}
)