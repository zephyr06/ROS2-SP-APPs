
include_directories("applications/tsp_solver_osm/include/")
include_directories("applications/tsp_solver_osm/src/")
include_directories("src/")
include_directories("include/")

file(GLOB SOURCES_TSP "src/imomd_rrt_star.cpp"
                        "src/eci_gen_tsp_solver.cpp" 
                        "src/osm_parser.cpp" 
                        "src/baseline/bi_a_star.cpp" 
                        "src/baseline/ana_star.cpp"
                        "src/tinyxml2/tinyxml2.cpp")
# file(GLOB SOURCES_TSP  "include/*.h"  "src/*.cpp")

add_executable(tsp_solver_executable_osm "main.cpp" ${SOURCES_TSP})
target_link_libraries(tsp_solver_executable_osm ${CONVENIENCE_LIB_NAME})

add_executable(tsp_solver_listener "tsp_listener.cpp" ${SOURCES_TSP})
target_link_libraries(tsp_solver_listener ${CONVENIENCE_LIB_NAME})