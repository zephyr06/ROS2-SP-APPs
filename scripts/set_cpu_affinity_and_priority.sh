# enable FIFO priority
sudo sysctl -w kernel.sched_rt_runtime_us=-1

dry_run=false
all_cfs=false

talker_name="talker"
slam_name="listener_slam"
rrt_name="rrt_listener"
mpc_name="listener_mpc"
tsp_name="tsp_solver_listener"

talker_cpu="0"
slam_cpu="1"
rrt_cpu="2"
mpc_cpu="1"
tsp_cpu="2"

# talker should always has the highest priority
talker_priority=10
slam_priority=2
rrt_priority=4
mpc_priority=5
tsp_priority=3

# talker related settings
talker_pids=$(ps -a -T | grep ${talker_name})
talker_output_list=($talker_pids)
cnt_output=${#talker_output_list[@]}
talker_tids=()
for i in $(seq 1 5 ${cnt_output}); do
    # echo ${talker_output_list[${i}]}
    talker_tids=(${talker_tids[@]} ${talker_output_list[${i}]})
done
echo "Set talker cpu affinity and priorities"
for tid in ${talker_tids[@]}; do
    echo "sudo taskset -c -p ${talker_cpu} ${tid}"
    if [ ${all_cfs} = true ] ; then
        echo "sudo chrt -o -p 0 ${tid}"
    else
        echo "sudo chrt -f -p ${talker_priority} ${tid}"
    fi
    if [ ${dry_run} = false ] ; then
        sudo taskset -c -p ${talker_cpu} ${tid}
        if [ ${all_cfs} = true ] ; then
            sudo chrt -o -p 0 ${tid}
        else
            sudo chrt -f -p ${talker_priority} ${tid}
        fi
    fi
done

# slam related settings
slam_pids=$(ps -a -T | grep ${slam_name})
slam_output_list=($slam_pids)
cnt_output=${#slam_output_list[@]}
slam_tids=()
for i in $(seq 1 5 ${cnt_output}); do
    # echo ${slam_output_list[${i}]}
    slam_tids=(${slam_tids[@]} ${slam_output_list[${i}]})
done
echo "Set slam cpu affinity and priorities"
for tid in ${slam_tids[@]}; do
    echo "sudo taskset -c -p ${slam_cpu} ${tid}"
    if [ ${all_cfs} = true ] ; then
        echo "sudo chrt -o -p 0 ${tid}"
    else
        echo "sudo chrt -f -p ${slam_priority} ${tid}"
    fi
    if [ ${dry_run} = false ] ; then
        sudo taskset -c -p ${slam_cpu} ${tid}
        if [ ${all_cfs} = true ] ; then
            sudo chrt -o -p 0 ${tid}
        else
            sudo chrt -f -p ${slam_priority} ${tid}
        fi
    fi
done

# rrt related settings
rrt_pids=$(ps -a -T | grep ${rrt_name})
rrt_output_list=($rrt_pids)
cnt_output=${#rrt_output_list[@]}
rrt_tids=()
for i in $(seq 1 5 ${cnt_output}); do
    # echo ${rrt_output_list[${i}]}
    rrt_tids=(${rrt_tids[@]} ${rrt_output_list[${i}]})
done
echo "Set rrt cpu affinity and priorities"
for tid in ${rrt_tids[@]}; do
    echo "sudo taskset -c -p ${rrt_cpu} ${tid}"
    if [ ${all_cfs} = true ] ; then
        echo "sudo chrt -o -p 0 ${tid}"
    else
        echo "sudo chrt -f -p ${rrt_priority} ${tid}"
    fi
    if [ ${dry_run} = false ] ; then
        sudo taskset -c -p ${rrt_cpu} ${tid}
        if [ ${all_cfs} = true ] ; then
            sudo chrt -o -p 0 ${tid}
        else
            sudo chrt -f -p ${rrt_priority} ${tid}
        fi
    fi
done

# mpc related settings
mpc_pids=$(ps -a -T | grep ${mpc_name})
mpc_output_list=($mpc_pids)
cnt_output=${#mpc_output_list[@]}
mpc_tids=()
for i in $(seq 1 5 ${cnt_output}); do
    # echo ${mpc_output_list[${i}]}
    mpc_tids=(${mpc_tids[@]} ${mpc_output_list[${i}]})
done
echo "Set mpc cpu affinity and priorities"
for tid in ${mpc_tids[@]}; do
    echo "sudo taskset -c -p ${mpc_cpu} ${tid}"
    if [ ${all_cfs} = true ] ; then
        echo "sudo chrt -o -p 0 ${tid}"
    else
        echo "sudo chrt -f -p ${mpc_priority} ${tid}"
    fi
    if [ ${dry_run} = false ] ; then
        sudo taskset -c -p ${mpc_cpu} ${tid}
        if [ ${all_cfs} = true ] ; then
            sudo chrt -o -p 0 ${tid}
        else
            sudo chrt -f -p ${mpc_priority} ${tid}
        fi
    fi
done

# tsp related settings
tsp_pids=$(ps -a -T | grep ${tsp_name})
tsp_output_list=($tsp_pids)
cnt_output=${#tsp_output_list[@]}
tsp_tids=()
for i in $(seq 1 5 ${cnt_output}); do
    # echo ${tsp_output_list[${i}]}
    tsp_tids=(${tsp_tids[@]} ${tsp_output_list[${i}]})
done
echo "Set tsp cpu affinity and priorities"
for tid in ${tsp_tids[@]}; do
    echo "sudo taskset -c -p ${tsp_cpu} ${tid}"
    if [ ${all_cfs} = true ] ; then
        echo "sudo chrt -o -p 0 ${tid}"
    else
        echo "sudo chrt -f -p ${tsp_priority} ${tid}"
    fi
    if [ ${dry_run} = false ] ; then
        sudo taskset -c -p ${tsp_cpu} ${tid}
        if [ ${all_cfs} = true ] ; then
            sudo chrt -o -p 0 ${tid}
        else
            sudo chrt -f -p ${tsp_priority} ${tid}
        fi
    fi
done