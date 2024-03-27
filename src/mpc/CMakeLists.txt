cmake_minimum_required(VERSION 3.5)
project(mpc)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add_subdirectory(src)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OsqpEigen)
find_package(Eigen3)

include_directories(${PROJECT_SOURCE_DIR}/../utils)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

#MPCExample
add_executable(MPCExample src/MPCExample.cpp)
target_link_libraries(MPCExample OsqpEigen::OsqpEigen)

add_executable(listener_mpc src/listener_mpc.cpp)
target_link_libraries(listener_mpc OsqpEigen::OsqpEigen)
ament_target_dependencies(listener_mpc  rclcpp std_msgs)

# find_package(OsqpEigen REQUIRED)
# find_package(Eigen3 3.1.0 REQUIRED)
# find_package(osqp REQUIRED)

# include_directories(
#   "/usr/local/include/osqp"
#   "/usr/local/share/osqp/codegen_files/inc/public"
#   "/usr/local/share/osqp/codegen_files/inc/private"
#   "/usr/include/OsqpEigen/"
#   ${EIGEN3_INCLUDE_DIR})

# add_executable(MPCExample src/example/src/MPCExample.cpp)
# ament_target_dependencies(MPCExample OsqpEigen rclcpp std_msgs osqp ${EIGEN3_LIBS})

install(TARGETS 
        MPCExample 
        listener_mpc 
        DESTINATION lib/${PROJECT_NAME})

ament_package()
