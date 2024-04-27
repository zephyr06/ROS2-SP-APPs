

file(GLOB SOURCES_TSP "src/Approx.cpp"
                  "src/BnB.cpp"
                  "src/Greedy.cpp"
                  "src/HC.cpp"
                  "src/Read.cpp"
                  "src/SA.cpp"
                  "src/TSP.h"
                  "src/TSPSolver.h"
                  "src/TSPSolver.cpp")

add_executable(tsp_solver_executable "src/TSP_executable.cpp" ${SOURCES_TSP})
target_link_libraries(tsp_solver_executable ${CONVENIENCE_LIB_NAME})

