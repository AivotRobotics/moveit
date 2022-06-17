set(MOVEIT_LIB_NAME moveit_collision_detection_acl)

add_library(${MOVEIT_LIB_NAME}
        src/collision_env_acl.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")


target_link_libraries(${MOVEIT_LIB_NAME} moveit_collision_detection ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

add_library(collision_detector_acl_plugin src/collision_detector_acl_plugin_loader.cpp)
set_target_properties(collision_detector_acl_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(collision_detector_acl_plugin ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} moveit_planning_scene)


install(TARGETS ${MOVEIT_LIB_NAME} collision_detector_acl_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(FILES ../collision_detector_acl_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
