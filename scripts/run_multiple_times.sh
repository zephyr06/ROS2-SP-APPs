# $1: repeat times
sleep_interval=120
repeat=20
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
    /bin/bash start_system.sh RM_Slow
    # python3 report_avg_scheduler_overhead.py
    sleep $sleep_interval
done



for i in $(seq 1 $repeat);
do
    echo $i
    /bin/bash start_system.sh CFS
    # python3 report_avg_scheduler_overhead.py
    sleep $sleep_interval
done


sleep $sleep_interval

for i in $(seq 1 $repeat);
do
    echo $i
    /bin/bash start_system.sh optimizerBF
    # python3 report_avg_scheduler_overhead.py
    sleep $sleep_interval
done

sleep $sleep_interval

for i in $(seq 1 $repeat);
do
    echo $i
    /bin/bash start_system.sh optimizerIncremental
    # python3 report_avg_scheduler_overhead.py
    sleep $sleep_interval
done
