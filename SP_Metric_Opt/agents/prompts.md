This is a research project about optimizing the safety-performance metric (SP) metric via scheduling algorithms.
This paper proposed 2 algorithms: INCR (sources/Optimization/OptimizeSP_TL_Incre.h) and BRUTE-FORCE (sources/Optimization/OptimizeSP_TL_BF.h).
optimization variables are tasks' priority assignments, and tasks' execution time running time limit.
Since the brute-force algorithm enumerates over all the possibile solution space and doesn't time out when the number of tasks is small, BR should have optimal performance.
however, during tests, INCR often shows better performance than BR. so i feel there should be some bugs or issues. i need you to understand why and find any coding issues.

All existing unit tests in the project passed.

Here is the unexpected tests:
"""
(base) zephyr@zephyr-Vostro-5490:~/Programming/ROS2-SP-APPs/SP_Metric_Opt/build$ ./tests/RunOrchestrator /home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/tests/comparison_runs/taskset_0_input /home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/tests/comparison_runs/taskset_0_output INCR 30000
Running Orchestrator: Input=/home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/tests/comparison_runs/taskset_0_input, Output=/home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/tests/comparison_runs/taskset_0_output, Mode=INCR, Duration=30000 ms
Average SP Metric: 2.77778
Saved average SP to: /home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/tests/comparison_runs/taskset_0_output/INCR/sp_metrics_summary.txt
(base) zephyr@zephyr-Vostro-5490:~/Programming/ROS2-SP-APPs/SP_Metric_Opt/build$ ./tests/RunOrchestrator /home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/tests/comparison_runs/taskset_0_input /home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/tests/comparison_runs/taskset_0_output BF 30000
Running Orchestrator: Input=/home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/tests/comparison_runs/taskset_0_input, Output=/home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/tests/comparison_runs/taskset_0_output, Mode=BF, Duration=30000 ms
Average SP Metric: 2.22222
Saved average SP to: /home/zephyr/Programming/ROS2-SP-APPs/SP_Metric_Opt/tests/comparison_runs/taskset_0_output/BF/sp_metrics_summary.txt
"""