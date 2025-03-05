// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
#include "sources/Optimization/OptimizeSP_TL_BF.h"

#include "sources/RTDA/ImplicitCommunication/ScheduleSimulation.h"
	
#include "sources/Utils/Parameters.h"

using ::testing::AtLeast;  // #1
using ::testing::Return;
using namespace std;
using namespace SP_OPT_PA;
using namespace GlobalVariables;

class TaskSetForTest_robotics_v19 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v19";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = ReadSP_Parameters(path);
    }

    // data members
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};

TEST_F(TaskSetForTest_robotics_v19, test_RM_FAST) {	
    //dag_tasks = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
    //                   "TaskData/test_robotics_v19.yaml");  // low utilization
    //std::string path = GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v19" + ".yaml";

		
    const TaskSet& tasks = dag_tasks.GetTaskSet();
    TaskSetInfoDerived tasks_info(tasks);
	
    // RYAN_HE: CSP simulation API
    Schedule s = SimulatedCSP_SingleCore(dag_tasks,tasks_info,sp_parameters,
                                         0, //processor_id,							
                                         1, // LLint simt,
                                         "RM_FAST", // std::string priority_policy,
                                         nullptr, // std::ofstream *fout,
                                         nullptr, // std::vector<std::vector<float>> *task_Ets=nullptr,
                                         1, // int output_job_not_executed=1,
                                         nullptr, // std::ofstream *flog=nullptr,
                                         10000 // int reevaluate_prio_interval_ms=10000		
								         );

    EXPECT_EQ(1,s.size());  // only scheduled 1 	
	auto itr = s.begin();
    JobCEC job = itr->first;
	EXPECT_EQ(0,job.taskId); // TSP is run first (smaller period)
	JobStartFinish sf = itr->second;
	EXPECT_EQ(400,sf.executionTime); // RM slow pick shotest to run						 
}

TEST_F(TaskSetForTest_robotics_v19, test_RM_SLOW) {	
    //dag_tasks = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
    //                   "TaskData/test_robotics_v19.yaml");  // low utilization
    //std::string path = GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v19" + ".yaml";

		
    const TaskSet& tasks = dag_tasks.GetTaskSet();
    TaskSetInfoDerived tasks_info(tasks);
	
    // RYAN_HE: CSP simulation API
    Schedule s = SimulatedCSP_SingleCore(dag_tasks,tasks_info,sp_parameters,
                                         0, //processor_id,							
                                         1, // LLint simt,
                                         "RM_SLOW", // std::string priority_policy,
                                         nullptr, // std::ofstream *fout,
                                         nullptr, // std::vector<std::vector<float>> *task_Ets=nullptr,
                                         1, // int output_job_not_executed=1,
                                         nullptr, // std::ofstream *flog=nullptr,
                                         10000 // int reevaluate_prio_interval_ms=10000		
								         );

    EXPECT_EQ(1,s.size());  // only scheduled 1 	
	auto itr = s.begin();
    JobCEC job = itr->first;
	EXPECT_EQ(0,job.taskId); // TSP is run first (smaller period)
	JobStartFinish sf = itr->second;
	EXPECT_EQ(1000,sf.executionTime); // RM slow pick longest to run								 
}

TEST_F(TaskSetForTest_robotics_v19, test_CFS) {	
    //dag_tasks = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
    //                   "TaskData/test_robotics_v19.yaml");  // low utilization
    //std::string path = GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v19" + ".yaml";

		
    const TaskSet& tasks = dag_tasks.GetTaskSet();
    TaskSetInfoDerived tasks_info(tasks);
	
    // RYAN_HE: CSP simulation API
    Schedule s = SimulatedCSP_SingleCore(dag_tasks,tasks_info,sp_parameters,
                                         0, //processor_id,							
                                         2, // LLint simt,
                                         "CFS", // std::string priority_policy,
                                         nullptr, // std::ofstream *fout,
                                         nullptr, // std::vector<std::vector<float>> *task_Ets=nullptr,
                                         1, // int output_job_not_executed=1,
                                         nullptr, // std::ofstream *flog=nullptr,
                                         10000 // int reevaluate_prio_interval_ms=10000		
								         );

    EXPECT_EQ(2,s.size());  // SHOULD SCHEDULED BOTH, 1 FOR EACH MS 	
	auto itr = s.begin();
    JobCEC job0 = itr->first;
	itr++;
    JobCEC job1 = itr->first;	
    int ok = 0;
	if (job0.taskId==0 && job1.taskId==3) {
		ok = 1;
	} else if (job0.taskId==3 && job1.taskId==0) {
		ok = 1;
	} 
	
	EXPECT_EQ(1,ok); // each one run 1 ms				 
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}