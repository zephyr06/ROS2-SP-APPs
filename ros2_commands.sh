# Build a single package
colcon build --packages-select cpp_pubsub --allow-overriding cpp_pubsub
colcon build --packages-select tsp_solver --allow-overriding tsp_solver
colcon build --packages-select mpc --allow-overriding mpc
source install/local_setup.bash

# Build all packages
colcon build 

# Run the publiser node
ros2 run cpp_pubsub talker