## Real-time Manager ROS2 Foxy

## Dependencies
yaml-cpp

## Nodes
- listener_scheduler:
    - The periodic scheduler task that will reschedule the system based the scheduler period.
    - Needs an input argument to define the scheduler, supported schedulers are: CFS, RM, optimizerBF, optimizerIncremental
    - For example: `ros2 run real_time_manager listener_schduler CFS`
- et_statistics: 
    - Estimate the execution time distribution for all four nodes {"tsp", "mpc", "rrt", "slam"} in the all_time_records folder.
    - Input argument "max_data_count" (optional, default is 50): only calculate distributions for the latest max_data_count entry.
    - Output file: all_time_records/task_characteristics.yaml.
- set_cpu_and_priority:
    - set cpu affinity and priority
    - Need sudo priority!!! any node using the `RealTimeManager` class will need priorities, otherwise it won't work.
        - Here is s simple but not convenient way to have access:
        1. Build all nodes
        1. Open a terminal and run `sudo -s`
        1. Source the setup bash: `source install/setup.bash`
        1. Run the needed node as a normal user
        1. !!! Only !!! Only use this way for the node will control priorities and cpu affinities. It might harm the system!
    - By default, it will read yaml file inside the `getTimeRecordFolder() + "cpu_and_priority.yaml"`
    - You may also specifiy a file to read priorities.
    - Supported scheduling_policy: `SCHED_OTHER`, `SCHED_FIFO`
    - Supported priority range:
        - `SCHED_OTHER`: [0,0] , this is CFS, only support priority level of 0.
        - `SCHED_FIFO` : [0,99], please use priorites starting from 1, the larger number, the higer priority.
    - The `talker` will be automatically added into the control list, with a priority higher than the max priority of other tasks (+5 is used). If all other tasks are CFS, then the `talker` is also CFS, otherwise, the `talker` will be FIFO.

## Example YAML files
- output of execution time statistics
```
tasks:
  - id: 0
    execution_time_mu: 10002.48852672
    execution_time_sigma: 3.133693975359407
    execution_time_min: 10000.973792
    execution_time_max: 10013.790912
    period: 10000
    deadline: 10000
    processorId: 0
    name: TSP
    sp_threshold: 0.5
    sp_Weight: 1
  - id: 1
    execution_time_mu: 58.14528960000001
    execution_time_sigma: 6.527760905237979
    execution_time_min: 12.756704
    execution_time_max: 60.698368
    period: 20
    deadline: 20
    processorId: 1
    name: MPC
    sp_threshold: 0.5
    sp_Weight: 1
  - id: 2
    execution_time_mu: 1128.0019616
    execution_time_sigma: 678.2485558057352
    execution_time_min: 204.776896
    execution_time_max: 3270.125888
    period: 2000
    deadline: 2000
    processorId: 1
    name: RRT
    sp_threshold: 0.5
    sp_Weight: 1
  - id: 3
    execution_time_mu: 60781.24272831999
    execution_time_sigma: 383724.0403412285
    execution_time_min: 538.48128
    execution_time_max: 2745169.19824
    period: 1000
    deadline: 1000
    processorId: 0
    name: SLAM
    sp_threshold: 0.5
    sp_Weight: 1
```

- input files for cpu affinities and priorities
```
tasks:
  - id: 0
    name: TSP
    processorId: 0
    scheduling_policy: SCHED_FIFO
    priority: 2
  - id: 1
    name: MPC
    processorId: 1
    scheduling_policy: SCHED_FIFO
    priority: 4
  - id: 2
    name: RRT
    processorId: 1
    scheduling_policy: SCHED_FIFO
    priority: 3
  - id: 3
    name: SLAM
    processorId: 0
    scheduling_policy: SCHED_FIFO
    priority: 1
```