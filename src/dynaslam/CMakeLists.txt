cmake_minimum_required(VERSION 3.5)
project(dynaslam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# find opencv
find_package(OpenCV 2.4.11 QUIET)
if(NOT OpenCV_FOUND)
    message("OpenCV > 2.4.11 not found.")
    find_package(OpenCV 3.0 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 3.0 not found.")
    endif()
endif()

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Test REQUIRED)

find_package(PythonLibs REQUIRED)
if (NOT PythonLibs_FOUND)
    message(FATAL_ERROR "PYTHON LIBS not found.")
else()
    message("PYTHON LIBS were found!")
    message("PYTHON LIBS DIRECTORY: " ${PYTHON_LIBRARY})
endif()

find_package(Boost REQUIRED COMPONENTS thread)
if(Boost_FOUND)
    message("Boost was found!")
    message("Boost Headers DIRECTORY: " ${Boost_INCLUDE_DIRS})
    message("Boost LIBS DIRECTORY: " ${Boost_LIBRARY_DIRS})
    message("Found Libraries: " ${Boost_LIBRARIES})
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

set(DYNASLAM_PROJ_DIR /home/nvidia/workspace/sdcard/SP_Scheduler_Stack/YOLO-DynaSLAM)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${DYNASLAM_PROJ_DIR}
  ${DYNASLAM_PROJ_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  /usr/include/python2.7/
  /usr/lib/python2.7/dist-packages/numpy/core/include/numpy/
  ${Boost_INCLUDE_DIRS}
)

add_executable(dynaslam_demo src/dynaslam_demo.cpp)

target_include_directories(dynaslam_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(dynaslam_demo 
  ${DYNASLAM_PROJ_DIR}/lib/libDynaSLAM.so
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${DYNASLAM_PROJ_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${DYNASLAM_PROJ_DIR}/Thirdparty/g2o/lib/libg2o.so
  /lib/aarch64-linux-gnu/libpython2.7.so
  ${Boost_LIBRARIES}
)

install(TARGETS dynaslam_demo
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
