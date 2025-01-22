
#include "sources/RTDA/ImplicitCommunication/ScheduleSimulation.h"
#include "sources/Utils/Parameters.h"

#if defined(RYAN_HE_CHANGE)
#include "sources/Optimization/OptimizeSP_Base.h"
#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"

#include <fstream>
#endif

namespace SP_OPT_PA {


void AddTasksToRunQueue(RUNQUEUE &run_queue, 
						const DAG_Model &dag_tasks,
                        int processor_id, LLint time_now) {
    for (int task_id = 0;
         task_id < static_cast<int>(dag_tasks.GetTaskSet().size()); task_id++) {
        Task task_curr = dag_tasks.GetTask(task_id);
        if (task_curr.processorId == processor_id &&
            time_now % task_curr.period == 0) {
            JobCEC job_curr(task_id, time_now / task_curr.period);
#if defined(RYAN_HE_CHANGE)
            // RYAN_HE: 
            // for RunQueue2, need to providd time_now when adding a job into run queue
            // it is needed to update job deadline and evaluate priority in run queue for EDF 
            // NOTE: if a deadline missed, the job index will also lost one in schedule output
            // i.e. if you see (1,0) start, end, exe, (1,2) start, end, exe,
            // that means job 1 of task 1 was not executed
            run_queue.insert(job_curr, time_now);
#else 
            run_queue.insert(job_curr);
#endif
        }
    }
}

#if defined(RYAN_HE_CHANGE)
// RYAN_HE: used by CSP scheduler to check if any new jobs should be added to run queue
// return number of new jobs added
// in case any new job is added, will need to run CSP scheduler (BR or incremental) to reassign
// priorities to all tasks
// 
// change to return vector of tasks added 
// the added task will need to be evaluated by CSP to decide execution time
std::vector<int> AddTasksToCSPRunQueue(RunQueueCSP &run_queue, 
	                                   const DAG_Model &dag_tasks,
                                       int processor_id, LLint time_now ) {
    std::vector<int> added_task_ids;
    for (int task_id = 0;
         task_id < static_cast<int>(dag_tasks.GetTaskSet().size()); task_id++) {
        Task task_curr = dag_tasks.GetTask(task_id);
        if (task_curr.processorId == processor_id &&  time_now % task_curr.period == 0) {
            // this is the new release time for this task
            if ( run_queue.is_task_running(task_id) == 0) { 
                // according to paper, new job is added only if previous one has finished 
                added_task_ids.push_back(task_curr.id);

                JobCEC job_curr(task_id, time_now / task_curr.period);

                // this is the randomly generated execution time for this job 
                // (following its distribution)
                double execution_time = task_curr.getExecutionTimeFromDist();
                int executionTime = int(execution_time+0.5);
#if defined(RYAN_HE_CHANGE_DEBUG)
                if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {
                    std::cout<<"####AddTasksToCSPRunQueue: task_id="<<task_id<<", jobId="<<job_curr.jobId;
                    std::cout<<", executionTime="<<executionTime<<std::endl;
                }
#endif
                // add it to run queue (the name insert maybe misleadding). We simply add it to the end
                // RunHigestPriority will pick the highest one to run anyway.
                run_queue.insert(job_curr, time_now, executionTime);
            }
        }
    }
    return added_task_ids;
}
#endif

Schedule SimulatedFTP_SingleCore(const DAG_Model &dag_tasks,
                                 const TaskSetInfoDerived &tasks_info,
                                 int processor_id		
#if defined(RYAN_HE_CHANGE)									
                                 , LLint simt				
#endif									 
								 ) {
    const TaskSet &tasks = dag_tasks.GetTaskSet();

#if defined(RYAN_HE_CHANGE)					
    // if simt not valid, using hyper period				
    if (simt<=0) 
        simt = tasks_info.hyper_period;
#endif	

    RUNQUEUE run_queue(tasks);
    if ( GlobalVariables::debugMode & DBG_PRT_MSK_RUNQUEUE ) {
      run_queue.dbg = 1;
    }

    for (LLint time_now = 0; time_now <= simt; time_now++) {
#if defined(RYAN_HE_CHANGE_DEBUG)
        if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {
            std::cout<<"\n\n####SimulatedFTP_SingleCore: time_now="<<time_now<<std::endl;
        }
#endif
        // first remove jobs that have been finished at this time
        run_queue.RemoveFinishedJob(time_now);

        // check whether to add new instances
        if (time_now < simt)
            AddTasksToRunQueue(run_queue, dag_tasks, processor_id, time_now);

        // Run jobs with highest priority
        run_queue.RunJobHigestPriority(time_now);
    }
    return run_queue.GetSchedule();
}

#if defined(RYAN_HE_CHANGE)

// RYAN_HE: main api for CSP simulation
// priority_policy: "BR" or "INCR"
Schedule SimulatedCSP_SingleCore_CSP(const DAG_Model &dag_tasks,
                                 const TaskSetInfoDerived &tasks_info,
                                 const SP_Parameters &sp_parameters,
                                 int processor_id,							
                                 LLint simt,
                                 std::string priority_policy,
                                 std::ofstream *fout								 
								 ) {

#if defined(RYAN_HE_CHANGE_DEBUG)
    std::string ppre="####SimulatedCSP_SingleCore_CSP: ";
#endif

    const TaskSet &tasks = dag_tasks.GetTaskSet();
							
    if (simt<=0) {
        simt = tasks_info.hyper_period;
    }

    // RYAN_HE:
    // set priority policy to assigned so that scheuler will not need to calculate priorities
    // based on RM or EDF etc
    // just let CSP calculate and assign priority
    // initial priority is assigned but it does not matter since it will be overwritten by CSP
    // when simulation starts 
    RunQueueCSP run_queue(tasks);
    for (int task_id = 0; task_id < tasks.size(); task_id++) {
        run_queue.set_task_priorityType(task_id, "assigned");
        run_queue.set_task_priority(task_id, tasks.size()-task_id-1);
    }

    if ( GlobalVariables::debugMode & DBG_PRT_MSK_RUNQUEUE ) {
      run_queue.dbg = 1;
    }    


    // RYAN_HE: for INCR only
    OptimizePA_Incre_with_TimeLimits inc_opt(dag_tasks, sp_parameters);

    // start simulation loop ...
    for (LLint time_now = 0; time_now <= simt; time_now++) {
#if defined(RYAN_HE_CHANGE_DEBUG)
        if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {
            std::cout<<"\n\n"<<ppre<<"time_now="<<time_now<<std::endl;
        }
#endif

        // first remove jobs that have been finished at this time
        run_queue.RemoveFinishedJob(time_now, fout);

        // RYAN_HE: check if any new jobs are added
        std::vector<int> added_task_ids;
        if (time_now < simt) {
            added_task_ids = AddTasksToCSPRunQueue(run_queue, dag_tasks, processor_id, time_now);
        }
        if (added_task_ids.size()>0) {
            // if any new jobs are added, we need to recalculate priorities
#if defined(RYAN_HE_CHANGE_DEBUG)                
            if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {
                std::cout << ppre << "added_task_ids.size()=" << added_task_ids.size() << ", recalc priority\n";
            }
#endif
            // we need to run BF or incremental alg to sort priorities
            ResourceOptResult res;
            if (priority_policy == "INCR") {
                if (time_now==0) {
                    PriorityVec pa_opt = inc_opt.OptimizeFromScratch_w_TL(
                        GlobalVariables::Layer_Node_During_Incremental_Optimization);
                } else {
                    PriorityVec pa_opt = inc_opt.OptimizeIncre_w_TL(dag_tasks,
                        GlobalVariables::Layer_Node_During_Incremental_Optimization);                    
                }
                res = inc_opt.CollectResults();
            } else {
                res = EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
            }

            // try to re-set its execution time since this task has different configuration
            for (int i=0; i<added_task_ids.size(); i++) {
                int task_id = added_task_ids[i];
                if ( res.id2time_limit[task_id] > 0 ) {
                    int newexetime = res.id2time_limit[task_id]; // RYAN_HE: MAY BE ADD SOME VARIATION ??
                    std::cout<< "new exe time " << newexetime << "\n";
                    double std = tasks_info.GetTask(task_id).getExecutionTimePerformanceSigma();
                    if (std>0.0) {
                        double pmin = tasks_info.GetTask(task_id).getExecutionTimePerformanceMin();
                        double pmax = tasks_info.GetTask(task_id).getExecutionTimePerformanceMax();
                        double *pminp = nullptr;
                        double *pmaxp = nullptr;
                        if (pmin>0.0 ) {
                            pminp = &pmin;
                        }
                        if (pmax>0.0 ) {
                            pmaxp = &pmax;
                        }
                        //int oldt = newexetime;
                        newexetime = int(getRandomValueByMuSigma(newexetime, std, pminp, pmaxp) + 0.5);
                        //std::cout << ppre << "TASK " << task_id << "'s EXE_TIME was: " << oldt << " -> " << newexetime << "\n";
                    }
                    run_queue.set_job_executionTime(task_id, newexetime); 
#if defined(RYAN_HE_CHANGE_DEBUG)           
                    if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {               
                        std::cout << ppre << "TASK " << task_id << "'s EXE_TIME set to: " << newexetime << "\n";
                    }
#endif
                }
            }

            PriorityVec pa_opt = res.priority_vec;

            // update priorities
            for (uint i = 0; i < pa_opt.size(); i++) {
                int task_i = pa_opt[i];
                int task_id = tasks[task_i].id;

                // this is old priority. get it for debug
                double priority = run_queue.tasks_info_.GetTask(task_id).get_priority();

                double new_priority = tasks.size()-task_id-1;
                run_queue.set_task_priority(task_id, new_priority);

#if defined(RYAN_HE_CHANGE_DEBUG)                
                if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {
                    std::string name = tasks[task_i].name;
                    std::cout << "    " << name << "," << task_id << "," << ": priority " << priority << " -> " << new_priority << "\n";

                    //verify
                    //double priority_new1 = run_queue.tasks_info_.GetTask(task_id).get_priority();
                    //if (priority_new1 != new_priority) {
                    //    std::cout << "Error in SimulatedCSP_SingleCore_CSP, return priority " <<priority_new1 << " != " << new_priority << "\n";
                    //}
                }
#endif
            }
        }

        // Run jobs with highest priority
        run_queue.RunJobHigestPriority(time_now);
    }

#if defined(RYAN_HE_CHANGE_DEBUG)   
    if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {
        std::cout<<"\n\n"<<ppre<<"DONE. schedule size="<<run_queue.GetSchedule().size()<<std::endl;
    }
#endif

    return run_queue.GetSchedule();
}

// RYAN_HE: main api for RM_FAST/SLOW simulation
// priority_policy: "RM_FAST" or "RM_SLOW"
Schedule SimulatedCSP_SingleCore_RM(const DAG_Model &dag_tasks,
                                 const TaskSetInfoDerived &tasks_info,
                                 const SP_Parameters &sp_parameters,
                                 int processor_id,							
                                 LLint simt,
                                 std::string priority_policy,
                                 std::ofstream *fout										 
								 ) {
#if defined(RYAN_HE_CHANGE_DEBUG)
    std::string ppre="####SimulatedCSP_SingleCore_RM: ";
#endif

    const TaskSet &tasks = dag_tasks.GetTaskSet();

    if (simt<=0) {
        simt = tasks_info.hyper_period;
    }

    // RYAN_HE:
    // set priority policy to assigned so that scheduler will not need to calculate priorities
    // just calcluate priorities at beginning
    // when simulation starts 
    RunQueueCSP run_queue(tasks);
    for (int task_id = 0; task_id < tasks.size(); task_id++) {
        run_queue.set_task_priorityType(task_id, "assigned");
        // larger period means smaller priority value (lower priority)
        double priority = 1.0/tasks[task_id].period - 1.0e-6*tasks[task_id].id;
        run_queue.set_task_priority(task_id, priority);
#if defined(RYAN_HE_CHANGE_DEBUG)
        if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {
            std::cout<<ppre<<"task_id="<<task_id<<", priority="<<priority<<std::endl;
        }
#endif        
    }

    if ( GlobalVariables::debugMode & DBG_PRT_MSK_RUNQUEUE ) {
      run_queue.dbg = 1;
    }

    // start simulation loop ...
    for (LLint time_now = 0; time_now <= simt; time_now++) {
#if defined(RYAN_HE_CHANGE_DEBUG)
        if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {
            std::cout<<"\n\n"<<ppre<<"time_now="<<time_now<<std::endl;
        }
#endif

        // first remove jobs that have been finished at this time
        run_queue.RemoveFinishedJob(time_now, fout);

        // RYAN_HE: check if any new jobs are added
        std::vector<int> added_task_ids;
        if (time_now < simt) {
            added_task_ids = AddTasksToCSPRunQueue(run_queue, dag_tasks, processor_id, time_now);
        }
        if (added_task_ids.size()>0) {
            // try to re-set its execution time (either min or max)
            for (int i=0; i<added_task_ids.size(); i++) {
                int task_id = added_task_ids[i];
                int newexetime;
                if ( priority_policy == "RM_FAST" ) {
                    newexetime = (int)(0.5+dag_tasks.GetTask(task_id).getExecutionTimeFromDistMin());
                } else { // ( priority_policy == "RM_SLOW" ) {
                    newexetime = (int)(0.5+dag_tasks.GetTask(task_id).getExecutionTimeFromDistMax());
                }
                run_queue.set_job_executionTime(task_id, newexetime); 
#if defined(RYAN_HE_CHANGE_DEBUG)          
                if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {                
                    std::cout << ppre << "TASK " << task_id << "'s EXETIME set to: " << newexetime << "\n";
                }
#endif
            }
        }

        // Run jobs with highest priority
        run_queue.RunJobHigestPriority(time_now);
    }

#if defined(RYAN_HE_CHANGE_DEBUG)   
    if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {
        std::cout<<"\n\n"<<ppre<<"DONE. schedule size="<<run_queue.GetSchedule().size()<<std::endl;
    }
#endif

    return run_queue.GetSchedule();
}

// RYAN_HE: main api for CFS simulation (FAIR)
Schedule SimulatedCSP_SingleCore_CFS(const DAG_Model &dag_tasks,
                                 const TaskSetInfoDerived &tasks_info,
                                 const SP_Parameters &sp_parameters,
                                 int processor_id,							
                                 LLint simt,
                                 std::string priority_policy,
                                 std::ofstream *fout												 
								 ) {
#if defined(RYAN_HE_CHANGE_DEBUG)
    std::string ppre="####SimulatedCSP_SingleCore_CFS: ";
#endif

    const TaskSet &tasks = dag_tasks.GetTaskSet();

    if (simt<=0) {
        simt = tasks_info.hyper_period;
    }

    // RYAN_HE:
    // set priority policy to assigned so that scheduler will not need to calculate priorities
    // priority not matter since we are doing round robin kind of scheduling
    RunQueueCSP run_queue(tasks);
    for (int task_id = 0; task_id < tasks.size(); task_id++) {
        run_queue.set_task_priorityType(task_id, "assigned");
        double priority = 0;
        run_queue.set_task_priority(task_id, priority);
#if defined(RYAN_HE_CHANGE_DEBUG)
        if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {
            std::cout<<ppre<<"task_id="<<task_id<<", priority="<<priority<<std::endl;
        }
#endif        
    }

    if ( GlobalVariables::debugMode & DBG_PRT_MSK_RUNQUEUE ) {
      run_queue.dbg = 1;
    }

    // start simulation loop ...
    for (LLint time_now = 0; time_now <= simt; time_now++) {
#if defined(RYAN_HE_CHANGE_DEBUG)
        if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {
            std::cout<<"\n\n"<<ppre<<"time_now="<<time_now<<std::endl;
        }
#endif

        // first remove jobs that have been finished at this time
        run_queue.RemoveFinishedJob(time_now, fout);

        // RYAN_HE: check if any new jobs are added
        std::vector<int> added_task_ids;
        if (time_now < simt) {
            added_task_ids = AddTasksToCSPRunQueue(run_queue, dag_tasks, processor_id, time_now);
        }
        if (added_task_ids.size()>0) {
            // we don't need to re-set any execution time
            // just use the one generated in AddTasksToCSPRunQueue 
        }

        // rotate and run next job
        run_queue.RotateRunNextJob(time_now);
    }

#if defined(RYAN_HE_CHANGE_DEBUG)   
    if (GlobalVariables::debugMode & DBG_PRT_MSK_SIMULATION) {
        std::cout<<"\n\n"<<ppre<<"DONE. schedule size="<<run_queue.GetSchedule().size()<<std::endl;
    }
#endif

    return run_queue.GetSchedule();
}


// RYAN_HE: main api for CSP simulation
Schedule SimulatedCSP_SingleCore(const DAG_Model &dag_tasks,
                                 const TaskSetInfoDerived &tasks_info,
                                 const SP_Parameters &sp_parameters,
                                 int processor_id,							
                                 LLint simt,
                                 std::string priority_policy,
                                 std::ofstream *fout										 
								 ) {

    if (priority_policy == "BR" || priority_policy == "INCR") {
        return SimulatedCSP_SingleCore_CSP(dag_tasks, tasks_info, sp_parameters, processor_id, 
                                           simt, priority_policy, fout);
    }
    else if (priority_policy == "RM_FAST" || priority_policy == "RM_SLOW") {
        return SimulatedCSP_SingleCore_RM(dag_tasks, tasks_info, sp_parameters, processor_id, 
                                          simt, priority_policy, fout);
    }    
    else if (priority_policy == "CFS") {
        return SimulatedCSP_SingleCore_CFS(dag_tasks, tasks_info, sp_parameters, processor_id, 
                                           simt, priority_policy, fout);
    }     
    else {
        std::cout << "Error in SimulatedCSP_SingleCore, unknown priority policy " << priority_policy << "\n";
        return Schedule();
    }
}

#endif // RYAN_HE_CHANGE

std::vector<int> GetProcessorIds(const DAG_Model &dag_tasks) {
    std::vector<int> processor_ids;
    std::unordered_set<int> id_record;
    processor_ids.reserve(dag_tasks.GetTaskSet().size());
    for (uint task_id = 0; task_id < dag_tasks.GetTaskSet().size(); task_id++) {
        int processor_id = dag_tasks.GetTask(task_id).processorId;
        if (id_record.find(processor_id) == id_record.end()) {
            id_record.insert(processor_id);
            processor_ids.push_back(processor_id);
        }
    }
    return processor_ids;
}

Schedule SimulateFixedPrioritySched(const DAG_Model &dag_tasks,
                                    const TaskSetInfoDerived &tasks_info	
#if defined(RYAN_HE_CHANGE)									
                                    , LLint simt				
#endif					
									) {
    Schedule schedule_all;
    schedule_all.reserve(tasks_info.length);
    std::vector<int> processor_ids = GetProcessorIds(dag_tasks);
    for (int processor_id : processor_ids) {
        Schedule schedule_curr =	
            SimulatedFTP_SingleCore(dag_tasks, tasks_info, processor_id
#if defined(RYAN_HE_CHANGE)		
			                        , simt
#endif
			                        );
        schedule_all.insert(schedule_curr.begin(), schedule_curr.end());
    }

    return schedule_all;
}

#if defined(RYAN_HE_CHANGE)
Schedule SimulateCSPSched(const DAG_Model &dag_tasks,
                          const TaskSetInfoDerived &tasks_info,	
                          const SP_Parameters &sp_parameters,						
                          LLint simt,
                          std::string priority_policy,
                          std::ofstream *fout		
					      ) {

    // RYAN_HE: better to use a map to store dags for each processor
    // so that BR will be much faster!
    // in such case, id must be re-set for dags on each processor
    // TBD
    int get_dags_for_diff_processor = 0;

    Schedule schedule_all;
    schedule_all.reserve(tasks_info.length);
    std::vector<int> processor_ids = GetProcessorIds(dag_tasks);
    for (int processor_id : processor_ids) {
        Schedule schedule_curr =	
            SimulatedCSP_SingleCore(dag_tasks, tasks_info, sp_parameters, processor_id, 
                                    simt, priority_policy, fout);
        schedule_all.insert(schedule_curr.begin(), schedule_curr.end());
    }

    return schedule_all;
}
#endif // RYAN_HE_CHANGE

}  // namespace SP_OPT_PA
