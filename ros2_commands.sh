# Build a single package
colcon build --packages-select cpp_pubsub --allow-overriding cpp_pubsub
colcon build --packages-select tsp_solver --allow-overriding tsp_solver
colcon build --packages-select mpc --allow-overriding mpc
colcon build --packages-select dynaslam --allow-overriding dynaslam

source install/local_setup.bash

# Build all packages
colcon build 

# Run the publiser node
ros2 run cpp_pubsub talker tsp 500
ros2 run cpp_pubsub talker rrt 1000
ros2 run cpp_pubsub talker mpc 1000

# Run the listener node
ros2 run tsp_solver tsp_solver_listener
ros2 run rrt_solver rrt_listener
ros2 run mpc listener_mpc
ros2 run dynaslam listener_slam

# download data via terminal
scp -r nvidia@192.168.1.29:~/workspace/sdcard/ROS2-SP-APPs/all_time_records /home/zephyr/Programming/ROS2-SP-APPs/all_time_records
# ssh
ssh nvidia@192.168.1.29