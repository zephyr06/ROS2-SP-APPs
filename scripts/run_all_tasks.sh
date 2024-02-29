
source install/local_setup.bash

ros2 run dynaslam listener_slam &&
ros2 run mpc listener_mpc &&
ros2 run tsp_solver tsp_solver_listener &&
ros2 run rrt_solver rrt_listener &&

sleep(5)


ros2 run cpp_pubsub talker tsp 1000 &&
ros2 run cpp_pubsub talker rrt 1000 &&
ros2 run cpp_pubsub talker slam 2000 &&
ros2 run cpp_pubsub talker mpc 1000 &&
