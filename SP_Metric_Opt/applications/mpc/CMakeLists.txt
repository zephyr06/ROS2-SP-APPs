

find_package(OsqpEigen)
find_package(Eigen3)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

#MPCExample
add_executable(MPCExample src/MPCExample.cpp)
target_link_libraries(MPCExample OsqpEigen::OsqpEigen)

add_executable(listener_mpc src/listener_mpc.cpp)
target_link_libraries(listener_mpc OsqpEigen::OsqpEigen)
target_link_libraries(listener_mpc ${CONVENIENCE_LIB_NAME})
