// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
#include "sources/Utils/Parameters.h"

#include "sources/TaskModel/RegularTasks.h"
#include "sources/Safety_Performance_Metric/Probability.h"

using ::testing::AtLeast;  // #1
using ::testing::Return;
using namespace std;
using namespace SP_OPT_PA;
using namespace GlobalVariables;

class TaskSetForTest_2tasks : public ::testing::Test {
   public:
    void SetUp() override {
        std::vector<Value_Proba> dist_vec1 = {
            Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
        std::vector<Value_Proba> dist_vec2 = {Value_Proba(4, 0.7),
                                              Value_Proba(5, 0.3)};
        tasks.push_back(Task(0, dist_vec1, 5, 5, 0));
        tasks.push_back(Task(1, dist_vec2, 12, 12, 1));

        sp_parameters = SP_Parameters(tasks);
    }

    // data members
    TaskSet tasks;
    SP_Parameters sp_parameters;
};

class TaskSetForTest_robotics_v20 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v20";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";        
        file_path = path;
        dag_tasks = ReadDAG_Tasks(path, 5);
        sp_parameters = SP_Parameters(dag_tasks);
    }

    // data members
    string file_path;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};

class TaskSetForTest_taskset_cfg_10_1_gen_1 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "taskset_characteristics_0";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/taskset_cfg_10_1_gen_1/" + file_name + ".yaml";
        file_path = path;
        //dag_tasks = ReadDAG_Tasks(path, 5);
        dag_tasks = ReadDAG_Tasks(path);
        sp_parameters = SP_Parameters(dag_tasks);
    }

    // data members
    string file_path;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};

class TaskSetForTest_taskset_cfg_4_5_gen_1 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "taskset_characteristics_0";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/taskset_cfg_4_5_gen_1/" + file_name + ".yaml";
        file_path = path;
        //dag_tasks = ReadDAG_Tasks(path, 5);
        dag_tasks = ReadDAG_Tasks(path);
        sp_parameters = SP_Parameters(dag_tasks);
    }

    // data members
    string file_path;
    DAG_Model dag_tasks;
    SP_Parameters sp_parameters;
    int N = dag_tasks.tasks.size();
};

TEST_F(TaskSetForTest_robotics_v20, RecordCloseTimeLimitOptions) {
    std::vector<std::vector<double>> time_limit_options =
        RecordCloseTimeLimitOptions(dag_tasks);
    EXPECT_EQ(184.1, time_limit_options[0][0]);
    EXPECT_EQ(397.5, time_limit_options[0][1]);

    EXPECT_EQ(-1, time_limit_options[1][0]);
    EXPECT_EQ(-1, time_limit_options[2][0]);
    EXPECT_EQ(-1, time_limit_options[3][0]);
}

class TaskSetForTest_robotics_v18 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v18";
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

TEST_F(TaskSetForTest_robotics_v18, optimize) {
    // ResourceOptResult res_opt =
    //     EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    opt.OptimizeFromScratch_w_TL(2);
    ResourceOptResult res_opt = opt.CollectResults();
    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);

    EXPECT_EQ(1000, res_opt.id2time_limit[0]);  // SLAM+TSP have low utilization
}

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
TEST_F(TaskSetForTest_robotics_v19, RecordCloseTimeLimitOptions) {
    std::vector<std::vector<double>> time_limit_options =
        RecordCloseTimeLimitOptions(dag_tasks);
    EXPECT_EQ(4, time_limit_options.size());     // 4 tasks
    EXPECT_EQ(2, time_limit_options[0].size());  // 2 options for TSP
    EXPECT_EQ(800, time_limit_options[0][0]);
    EXPECT_EQ(1000, time_limit_options[0][1]);

    EXPECT_EQ(-1, time_limit_options[1][0]);
    EXPECT_EQ(-1, time_limit_options[2][0]);
    EXPECT_EQ(-1, time_limit_options[3][0]);
}
TEST_F(TaskSetForTest_robotics_v19, OptimizeFromScratch_w_TL) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    EXPECT_FALSE(opt.IfInitialized());
    opt.OptimizeFromScratch_w_TL(2);
    EXPECT_TRUE(opt.IfInitialized());
    ResourceOptResult res_opt = opt.CollectResults();
    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);
    EXPECT_EQ(400,
              res_opt.id2time_limit[0]);  // SLAM+TSP have high utilization;
    // In scratch mode, should return 400
    // In incremental mode, should return 800
}

TEST_F(TaskSetForTest_robotics_v19, optimize_incremental) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks,
                                         sp_parameters);  // high utilization

    opt.OptimizeFromScratch_w_TL(2);  // result is 400
    ResourceOptResult res_opt = opt.CollectResults();
    EXPECT_EQ(400,
              res_opt.id2time_limit[0]);  // SLAM+TSP have high utilization;

    DAG_Model dag_tasks_updated =
        ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                      "TaskData/test_robotics_v21.yaml");  // low utilization
    opt.OptimizeIncre_w_TL(dag_tasks_updated, 2);
    res_opt = opt.CollectResults();
    EXPECT_EQ(
        600,
        res_opt.id2time_limit[0]);  // change only to nearby ET level each time

    auto start_time = CurrentTimeInProfiler;
    for (int i = 0; i < 10; i++) opt.OptimizeIncre_w_TL(dag_tasks_updated, 2);
    auto finish_time = CurrentTimeInProfiler;
    double time_taken = GetTimeTaken(start_time, finish_time);
    EXPECT_LT(time_taken / 10.0,
              5e-2);  // since no adjustemnts are made, it should be very fast

    // dag_tasks_updated =
    //     ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
    //                   "TaskData/test_robotics_v22.yaml");  // low utilization
}

TEST_F(TaskSetForTest_taskset_cfg_10_1_gen_1, optimize_incremental) {
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks,sp_parameters);  

    int n =GlobalVariables::Layer_Node_During_Incremental_Optimization;
    std::cout<<"Layer_Node_During_Incremental_Optimization:" << n << std::endl;    
    auto start_time = CurrentTimeInProfiler;
    opt.OptimizeFromScratch_w_TL(n);
    ResourceOptResult res_opt = opt.CollectResults();
    auto finish_time = CurrentTimeInProfiler;
    double time_taken = GetTimeTaken(start_time, finish_time);
    std::cout<<"time taken for OptimizeFromScratch_w_TL:" << time_taken << std::endl;

    

    start_time = CurrentTimeInProfiler;
    opt.OptimizeIncre_w_TL(dag_tasks, n);
    finish_time = CurrentTimeInProfiler;
    time_taken = GetTimeTaken(start_time, finish_time);
    std::cout<<"time taken for OptimizeIncre_w_TL:" << time_taken << std::endl;
}

TEST_F(TaskSetForTest_taskset_cfg_4_5_gen_1, check_id2time_limit) {
    // simulate from 0 to 300s, cpu util should increase and then descrease
	// one task_characteristics file for every 10s 
    // check task 3 (with performance_records_time) exetime
    int task_wPerf = 3; // this task has performance_records_time
    
    std::cout<<"\n-------- check id2time_limit varies with cpu utilization\n";    
    
    
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks,sp_parameters);  
    int n = GlobalVariables::Layer_Node_During_Incremental_Optimization;

    // time 0: 
    opt.OptimizeFromScratch_w_TL(n);
    ResourceOptResult res_opt = opt.CollectResults();
	// calculate cpu_util for other tasks
    double cpu_util = 0.0;
    for ( int i=0;i<4;i++ ){
        if (i==task_wPerf) continue;
        const Task t = dag_tasks.GetTask(i);
        int prd = t.period;
        GaussianDist g_et = t.getExecGaussian();
        double mu = g_et.mu;
        cpu_util += mu/prd;
    }
	// print cpu_util, and id2time_limit for task 3
    printf("%02d: cpu_util_for_other_tasks=%.4f, exe_time_for_task_%d=%.4f\n",0, 
        cpu_util,task_wPerf,res_opt.id2time_limit[task_wPerf]); 
    
	// for the rest 30*10s, print cpu_util and res_opt.id2time_limit[task_wPerf]
	// we expect res_opt.id2time_limit[task_wPerf] increase/descrease while cpu_util descrease/increase
    for (int k=1;k<=30;k++) {        
        std::string file_name = "taskset_characteristics_" + std::to_string(k);
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/taskset_cfg_4_5_gen_1/" + file_name + ".yaml";
        DAG_Model dag_tasks_updated = ReadDAG_Tasks(path);    
        opt.OptimizeIncre_w_TL(dag_tasks_updated, n);
        res_opt = opt.CollectResults();
        
        cpu_util = 0.0;
        for ( int i=0;i<4;i++ ){
            if (i==task_wPerf) continue;
            const Task t = dag_tasks_updated.GetTask(i);
            int prd = t.period;
            GaussianDist g_et = t.getExecGaussian();
            double mu = g_et.mu;
            cpu_util += mu/prd;
        }        
        printf("%02d: cpu_util_for_other_tasks=%.4f, exe_time_for_task_%d=%.4f\n",k, 
            cpu_util,task_wPerf,res_opt.id2time_limit[task_wPerf]);         
    }
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}