# $1: repeat times
sleep_interval=120
repeat=10


for i in $(seq 1 $repeat);
do
    echo $i
    /bin/bash start_system.sh CFS
    # python3 report_avg_scheduler_overhead.py
    sleep $sleep_interval
done


for i in $(seq 1 $repeat);
do
    echo $i
    /bin/bash start_system.sh RM_Fast
    # python3 report_avg_scheduler_overhead.py
    sleep $sleep_interval
done


for i in $(seq 1 $repeat);
do
    echo $i
    /bin/bash start_system.sh optimizerBF
    # python3 report_avg_scheduler_overhead.py
    sleep $sleep_interval
done

for i in $(seq 1 $repeat);
do
    echo $i
    /bin/bash start_system.sh RM_Slow
    # python3 report_avg_scheduler_overhead.py
    sleep $sleep_interval
done

for i in $(seq 1 $repeat);
do
    echo $i
    /bin/bash start_system.sh optimizerIncremental
    # python3 report_avg_scheduler_overhead.py
    sleep $sleep_interval
done



PROJECT_ROOT="/home/nvidia/workspace/sdcard/ROS2-SP-APPs"
cd $PROJECT_ROOT/Experiments

current_time=$(date +"%Y%m%d%H%M%S")
backup_file_name=experiments_all_$1_${current_time}

mkdir ${backup_file_name}
cp -r CFS ${backup_file_name}/
cp -r RM_Fast ${backup_file_name}/
cp -r RM_Slow ${backup_file_name}/
cp -r optimizerBF ${backup_file_name}/
cp -r optimizerIncremental ${backup_file_name}/
tar -czf ${backup_file_name}.tar.gz ${backup_file_name}

# cp ${backup_file_name}.tar.gz ../Experiments/MultiRun/$1/
# mv ${backup_file_name}.tar.gz ../Experiments/$1/
# cp ${backup_file_name}/all_time_records_$1/current_scheduler_SP.pdf ../Experiments/$1/current_scheduler_SP_$1_${current_time}.pdf
rm ${backup_file_name} -r
