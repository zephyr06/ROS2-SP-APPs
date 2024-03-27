## Real-time Manager ROS2 Foxy

## Dependencies
yaml-cpp

## Nodes
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
    execution_time_mu: 1.00295837504
    execution_time_sigma: 0.005780039026146719
    execution_time_min: 1.00098448
    execution_time_max: 1.031703328
    period: 1.031703328
    deadline: 1.031703328
    name: TSP
  - id: 1
    execution_time_mu: 0.06105002047999999
    execution_time_sigma: 0.01247929785365278
    execution_time_min: 0
    execution_time_max: 0.066399712
    period: 0.066399712
    deadline: 0.066399712
    name: MPC
  - id: 2
    execution_time_mu: 1.4963107392
    execution_time_sigma: 0.8355111899446598
    execution_time_min: 0.3993056
    execution_time_max: 4.558432544
    period: 4.558432544
    deadline: 4.558432544
    name: RRT
  - id: 3
    execution_time_mu: 3.290805757440001
    execution_time_sigma: 4.336259460639159
    execution_time_min: 1.174386528
    execution_time_max: 17.979726016
    period: 17.979726016
    deadline: 17.979726016
    name: SLAM
```

- input files for cpu affinities and priorities
```
tasks:
  - id: 0
    name: TSP
    cpu_lists: [1,2]
    scheduling_policy: SCHED_FIFO
    priority: 2
  - id: 1
    name: MPC
    cpu_lists: [1]
    scheduling_policy: SCHED_FIFO
    priority: 4
  - id: 2
    name: RRT
    cpu_lists: [1]
    scheduling_policy: SCHED_FIFO
    priority: 3
  - id: 3
    name: SLAM
    cpu_lists: [1]
    scheduling_policy: SCHED_FIFO
    priority: 1
```