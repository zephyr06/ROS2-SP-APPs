cmake_minimum_required(VERSION 3.8)
project(tsp_solver)

set (CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${PROJECT_SOURCE_DIR}/../utils)
add_definitions(-DPROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}")
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()




file(GLOB SOURCES "src/Approx.cpp"
                  "src/BnB.cpp"
                  "src/Greedy.cpp"
                  "src/HC.cpp"
                  "src/Read.cpp"
                  "src/SA.cpp"
                  "src/TSP.h"
                  "src/TSPSolver.h"
                  "src/TSPSolver.cpp")
# file (GLOB SOURCES "src/*.cpp" "src/*.h")
add_executable(tsp_solver_listener src/listener_ros2.cpp ${SOURCES})
ament_target_dependencies(tsp_solver_listener rclcpp std_msgs)

add_executable(tsp_solver_executable "src/TSP_executable.cpp" ${SOURCES})



install(TARGETS
  tsp_solver_listener
  tsp_solver_executable
  DESTINATION lib/${PROJECT_NAME})
ament_package()
