export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH:/home/nvidia/workspace/sdcard/SP_Scheduler_Stack/YOLO-DynaSLAM/lib:/usr/local/lib

# enable FIFO priority
sudo sysctl -w kernel.sched_rt_runtime_us=-1

source /home/nvidia/workspace/sdcard/ROS2-SP-APPs/install/setup.bash

python clear_all_time_records.py
ros2 launch launch_all_the_nodes.py

# python analyze_response_time.py
# ./~/workspace/sdcard/SP_Metric_Opt/analyze_sp.sh
