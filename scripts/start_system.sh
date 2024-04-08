### check input validity
if [ $# -eq 0 ]; then
    echo "No arguments provided. A scheduler is needed: CFS, RM, optimizerBF, optimizerIncremental"
    echo "Exiting the script ..."
    exit 0
fi
if [ $1 != "RM" ] && [ $1 != "CFS" ] && [ $1 != "optimizerBF" ] && [ $1 != "optimizerIncremental" ] ; then
    echo "Scheduler $1 is not supported."
    echo "Supported schedulers are: CFS, RM, optimizerBF, optimizerIncremental"
    echo "Exiting the script ..."
    exit 0
fi

### prepare for nodes execution
export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH:/home/nvidia/workspace/sdcard/SP_Scheduler_Stack/YOLO-DynaSLAM/lib:/usr/local/lib
# enable FIFO priority
sudo sysctl -w kernel.sched_rt_runtime_us=-1

source ../install/setup.bash
python clear_all_time_records.py

# launch the ROS2 stack with specified schduler
ros2 launch launch_all_the_nodes.py scheduler:=$1

### post-execution operations, plot SP for the single scheudler, and backup data
python3 ../SP_Metric_Opt/Visualize_SP_Metric/draw_SP_current_scheduler.py $1

current_time=$(date +"%Y%m%d%H%M%S")
backup_file_name=all_time_records_$1_${current_time}
echo "backup in Experiments/$1/${backup_file_name}.tar.gz"

mkdir ${backup_file_name}
cp -r ../all_time_records ${backup_file_name}/all_time_records_$1
tar -czf ${backup_file_name}.tar.gz ${backup_file_name}

mv ${backup_file_name}.tar.gz ../Experiments/$1/
cp ${backup_file_name}/all_time_records_$1/current_scheduler_SP.pdf ../Experiments/$1/current_scheduler_SP_$1_${current_time}.pdf
rm ${backup_file_name} -r

