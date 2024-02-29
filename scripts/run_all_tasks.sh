
source install/local_setup.bash


python clear_all_time_records.py

# in miliseconds
slam_period=1000
rrt_period=1000
tsp_period=1000
mpc_period=1000



# ros2 run dynaslam listener_slam &
# ros2 run mpc listener_mpc &
# ros2 run tsp_solver tsp_solver_listener &
# ros2 run rrt_solver rrt_listener &
# ros2 run cpp_pubsub talker tsp $tsp_period &
# ros2 run cpp_pubsub talker rrt $rrt_period &
# ros2 run cpp_pubsub talker slam $slam_period &
# ros2 run cpp_pubsub talker mpc $mpc_period &
