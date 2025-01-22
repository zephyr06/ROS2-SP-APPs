
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

include_directories(${PROJECT_SOURCE_DIR}/../utils)
set(DYNASLAM_PROJ_DIR /home/nvidia/workspace/sdcard/SP_Scheduler_Stack/YOLO-DynaSLAM)
#set(ORBASLAM2_PROJ_DIR /home/nvidia/workspace/sdcard/ORB_SLAM2)
set(ORBASLAM2_PROJ_DIR /mnt/d/robotics_research/ORB_SLAM2)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${ORBASLAM2_PROJ_DIR}
  ${ORBASLAM2_PROJ_DIR}/include
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
  # ${DYNASLAM_PROJ_DIR}/lib/libDynaSLAM.so
  ${ORBASLAM2_PROJ_DIR}/lib/libORB_SLAM2.so
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORBASLAM2_PROJ_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORBASLAM2_PROJ_DIR}/Thirdparty/g2o/lib/libg2o.so
  /usr/lib/x86_64-linux-gnu/libpython2.7.so
  ${Boost_LIBRARIES}
)
target_link_libraries(dynaslam_demo ${CONVENIENCE_LIB_NAME})

add_executable(listener_slam src/listener_slam.cpp)
target_include_directories(listener_slam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(listener_slam 
  # ${DYNASLAM_PROJ_DIR}/lib/libDynaSLAM.so
  ${ORBASLAM2_PROJ_DIR}/lib/libORB_SLAM2.so
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORBASLAM2_PROJ_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORBASLAM2_PROJ_DIR}/Thirdparty/g2o/lib/libg2o.so
  /usr/lib/x86_64-linux-gnu/libpython2.7.so
  ${Boost_LIBRARIES}
)

target_link_libraries(listener_slam ${CONVENIENCE_LIB_NAME})
