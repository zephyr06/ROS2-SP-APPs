

add_executable(rrt_demo_withGUI src/rrt_demo_withGUI.cpp)
target_include_directories(rrt_demo_withGUI PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(rrt_demo_withGUI sfml-graphics sfml-window sfml-system)
target_link_libraries(rrt_demo_withGUI ${CONVENIENCE_LIB_NAME})



add_executable(rrt_demo_withoutGUI src/rrt_demo_withoutGUI.cpp)
target_include_directories(rrt_demo_withoutGUI PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(rrt_demo_withoutGUI sfml-graphics sfml-window sfml-system)
target_link_libraries(rrt_demo_withoutGUI ${CONVENIENCE_LIB_NAME})




# Add RRT-listener
add_executable(rrt_listener src/listener_rrt.cpp ${SOURCES})
target_include_directories(rrt_listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rrt_listener sfml-graphics sfml-window sfml-system)
target_link_libraries(rrt_listener ${CONVENIENCE_LIB_NAME})


