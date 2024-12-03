# $1: repeat times
sleep_interval=120
repeat=20

PROJECT_ROOT="/home/nvidia/workspace/sdcard/ROS2-SP-APPs"
cd $PROJECT_ROOT/scripts

schedulers=("CFS" "RM_Fast" "optimizerBF" "RM_Slow" "optimizerIncremental")
# schedulers=("optimizerBF")
for scheduler in "${schedulers[@]}"; do
    echo $scheduler
    rm -rf $PROJECT_ROOT/Experiments/$scheduler/*
    for i in $(seq 1 $repeat);
    do
        echo $i
        /bin/bash start_system.sh $scheduler
        sleep $sleep_interval
    done

done

cd $PROJECT_ROOT/Experiments

current_time=$(date +"%Y%m%d%H%M%S")
backup_file_name=experiments_all_$1_${current_time}

mkdir ${backup_file_name}
for scheduler in "${schedulers[@]}"; do
    cp -r $scheduler ${backup_file_name}/
done 

tar -czf ${backup_file_name}.tar.gz ${backup_file_name}
rm ${backup_file_name} -r
