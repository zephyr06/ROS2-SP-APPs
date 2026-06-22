High-level Task list:
- [x] Fix opt_sp_ initialization bug, instead of initializing it as 0, we'll use -1.
- [x] change configuration optimization's iteration logic. current implemntation iterates through all the possibile combination of execution time limit configurations of all tasks, which has expoentnial run-time complexity. We'll switch it to linear iteration by checking configuration of each task one by one. implement task sorting before iterating on tasks. related commit: 3b158b51c0dc88df38f0b6670173d781731ec6bc
- [x] add complete fairness scheduler simulation code, and related tests. check code in "simulation_exp" branch for reference, but notice that code in that branch has bugs and possible logic issues. so be cautious about what to take, and follow TDD.
- [x] add python simulated training data generator to generate simulated task sets, add core functionality and related tests (check simulation_exp branch):
    - [x] add python simulated environment that evaluates different schedulers' performance on the simulated dataset
    - [x] add pipeline script to generate python simulated task set from given configuration files end-to-end
- add ablation study baseline (check simulation_exp branch):
    * [x] INCR_NO_TL
    * [x] INCR_WCET
- [x] add GP-based ET distribution prediction, if needed.
- [x] add end-to-end task set execution schedule code in c++. the code should read from each individual task set at different time stamps, then run different scheduler method, and monitor overall SP metric (check simulation_exp branch for basic ideas of related code)
- add any missed baselines for simulation orchestrator:
* CFS (SimulateCFSSched has some implemntation code in sources/RTDA/ImplicitCommunication/ScheduleSimulation.h)
* ablation methods, INCR_NO_TL (GlobalVariables::disable_time_limit_opt) and INCR_WCET (GlobalVariables::use_wcet_execution_time)
* RM-Fast (TL optimization always uses the shortest time limit), RM-Slow (TL optimization always uses the longest time limit)

- add more types of reulst visualization figure plots(check simulation_exp branch):
* x axis: number of tasks per task set
* y axis:
    - figure 1: sp metric (differnt schedulers)
    - figure 2 : scheduler decision execution time
    - figure 3: average sp metric (only INCR scheduler, consider different optimizer invocation intervals)
    - figure 4: deadline miss rate of important tasks
- add end-to-end script to run simulation evaluation of all baseline methods for given task set config.
- other functional changes implemented in simulation_exp branch but not yet in this branch.

