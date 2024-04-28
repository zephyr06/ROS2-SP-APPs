

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/../utils)



add_executable(et_statistics src/et_statistics.cpp)
target_include_directories(et_statistics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(et_statistics ${CONVENIENCE_LIB_NAME})

add_executable(set_cpu_and_priority src/set_cpu_and_priority.cpp)
target_include_directories(set_cpu_and_priority PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(set_cpu_and_priority ${CONVENIENCE_LIB_NAME})


add_executable(update_priority_assignments src/update_priority_assignments.cpp)
target_include_directories(update_priority_assignments PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(update_priority_assignments ${CONVENIENCE_LIB_NAME})


add_executable(listener_scheduler src/listener_scheduler.cpp)
target_include_directories(listener_scheduler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(listener_scheduler ${CONVENIENCE_LIB_NAME})


