#pragma once

#ifdef RYAN_HE_CHANGE

// RYAN_HE: RunQueue2 replacing RunQueue for old RM/EDF type scheduling
#include "sources/RTDA/ImplicitCommunication/RunQueue2.h"
#define RUNQUEUE RunQueue2

// RunQueueCSP is for new CSP type scheduling
// NOTE: should unify them later if having time
#include "sources/RTDA/ImplicitCommunication/RunQueueCSP.h"

#include "sources/Safety_Performance_Metric/ParametersSP.h"

#else
#include "sources/RTDA/ImplicitCommunication/RunQueue.h"
#define RUNQUEUE RunQueue
#endif

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/colormod.h"



namespace SP_OPT_PA {

void AddTasksToRunQueue(RUNQUEUE &run_queue, const DAG_Model &dag_tasks,
                        int processor_id, LLint time_now);

// RYAN_HE: simt is the simulated time (in ms)
// if 0, using hyper period
// previous API always using hyper period which is ok for fixed execution time						
Schedule SimulatedFTP_SingleCore(const DAG_Model &dag_tasks,
                                 const TaskSetInfoDerived &tasks_info,
                                 int processor_id
#ifdef RYAN_HE_CHANGE
                                 , LLint simt=0
#endif										 
								 );

Schedule SimulateFixedPrioritySched(const DAG_Model &dag_tasks,
                                    const TaskSetInfoDerived &tasks_info
#ifdef RYAN_HE_CHANGE
                                    , LLint simt=0
#endif											
									);

#if defined(RYAN_HE_CHANGE)
// RYAN_HE: CSP simulation API
Schedule SimulatedCSP_SingleCore(const DAG_Model &dag_tasks,
                                 const TaskSetInfoDerived &tasks_info,
                                 const SP_Parameters &sp_parameters,
                                 int processor_id,							
                                 LLint simt,
                                 std::string priority_policy,
                                 std::ofstream *fout							 
								 );

Schedule SimulateCSPSched(const DAG_Model &dag_tasks,
                          const TaskSetInfoDerived &tasks_info,		
                          const SP_Parameters &sp_parameters,					
                          LLint simt,
                          std::string priority_policy,
                          std::ofstream *fout		
					      );                                 
#endif

}  // namespace SP_OPT_PA
