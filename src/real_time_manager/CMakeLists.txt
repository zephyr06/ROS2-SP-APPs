cmake_minimum_required(VERSION 3.5)
project(real_time_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

set (CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Release)

include_directories(${PROJECT_SOURCE_DIR}/../utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp 0.5.2 REQUIRED)

add_executable(et_statistics src/et_statistics.cpp)
target_include_directories(et_statistics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(et_statistics yaml-cpp)

add_executable(set_cpu_and_priority src/set_cpu_and_priority.cpp)
target_include_directories(set_cpu_and_priority PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(set_cpu_and_priority yaml-cpp)


add_executable(update_priority_assignments src/update_priority_assignments.cpp)
target_include_directories(update_priority_assignments PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(update_priority_assignments yaml-cpp)


add_executable(listener_scheduler src/listener_scheduler.cpp)
ament_target_dependencies(listener_scheduler rclcpp std_msgs)
target_include_directories(listener_scheduler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(listener_scheduler yaml-cpp)


add_executable(listener_scheduler_fixed_priority src/listener_scheduler_fixed_priority.cpp)
ament_target_dependencies(listener_scheduler_fixed_priority rclcpp std_msgs)
target_include_directories(listener_scheduler_fixed_priority PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(listener_scheduler_fixed_priority yaml-cpp)


install(TARGETS et_statistics set_cpu_and_priority update_priority_assignments listener_scheduler listener_scheduler_fixed_priority
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
