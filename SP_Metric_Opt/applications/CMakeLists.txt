find_package(OsqpEigen QUIET)
if(OsqpEigen_FOUND)
  add_subdirectory(mpc)
endif()

if(EXISTS "/home/nvidia/workspace/sdcard/ORB_SLAM2")
  find_package(Pangolin QUIET)
  if(Pangolin_FOUND)
    add_subdirectory(dynaslam)
  endif()
endif()

find_package(SFML 2 COMPONENTS graphics window system QUIET)
if(SFML_FOUND)
  add_subdirectory(rrt_solver)
endif()

add_subdirectory(real_time_manager)
add_subdirectory(tsp_solver_osm)