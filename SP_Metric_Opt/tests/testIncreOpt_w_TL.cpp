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
        //sp_parameters = SP_Parameters(dag_tasks);
        sp_parameters = ReadSP_Parameters(path);
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
        //sp_parameters = SP_Parameters(dag_tasks);
        sp_parameters = ReadSP_Parameters(path);
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

class TaskSetForTest_robotics_v19_2 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v19_2";
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

TEST_F(TaskSetForTest_robotics_v19_2, RecordCloseTimeLimitOptions) {
    printf("\n-------- TaskSetForTest_robotics_v19_2, RecordCloseTimeLimitOptions ...\n"); 
    std::vector<std::vector<double>> time_limit_options = RecordCloseTimeLimitOptions(dag_tasks);
    
    EXPECT_EQ(4, time_limit_options.size());     // 4 tasks

    int perfTask = 0;
    for (int i=0;i<time_limit_options.size();i++) {
        if (time_limit_options[i][0]!=-1) {
            perfTask = i;
            break;
        }
    }
    
    EXPECT_EQ(2, time_limit_options[perfTask].size());  // 2 options for TSP
    EXPECT_EQ(800, time_limit_options[perfTask][0]);
    EXPECT_EQ(1000, time_limit_options[perfTask][1]);

    for (int i=0;i<time_limit_options.size();i++) {
        if (i==perfTask) continue;
        EXPECT_EQ(-1, time_limit_options[i][0]);
    }  
}

TEST_F(TaskSetForTest_robotics_v19_2, OptimizeFromScratch_w_TL) {
    // NOTE: this test failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // compare with the testcase (TaskSetForTest_robotics_v19) before, the only difference is that 
    // this testcase added two tasks with very small cpu_utilization and big weight
    // supposedly, OptimizeFromScratch_w_TL should also return 400 for task0

    // try to get which task has performance_records_time
    int perfTask = 0;
    std::vector<std::vector<double>> time_limit_options = RecordCloseTimeLimitOptions(dag_tasks);
    for (int i=0;i<time_limit_options.size();i++) {
        if (time_limit_options[i][0]!=-1) {
            perfTask = i;
            break;
        }
    }
        
    printf("\n-------- TaskSetForTest_robotics_v19_2, OptimizeFromScratch_w_TL ...\n");     
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
    EXPECT_FALSE(opt.IfInitialized());

    //int n = dag_tasks.tasks.size();
    //for (int i=0;i<n;i++){
    //    double exeT = dag_tasks.GetTask(i).getExecGaussian().mu;
    //    const_cast<SP_OPT_PA::Task&>(dag_tasks.GetTask(i)).setExecutionTime(exeT);
    //    printf("task%d: exeT=%f\n",i,(double)(dag_tasks.GetTask(i).getExecutionTime()));        
    //}
    opt.OptimizeFromScratch_w_TL(2);
    EXPECT_TRUE(opt.IfInitialized());
    ResourceOptResult res_opt = opt.CollectResults();
    PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);
    EXPECT_EQ(400, res_opt.id2time_limit[perfTask]);  // SLAM+TSP have high utilization;
}

TEST_F(TaskSetForTest_robotics_v19_2, optimize_incremental) {
    printf("\n-------- TaskSetForTest_robotics_v19_2, optimize_incremental ...\n");     
    int granuality = 1;
    int n_options = 2;
    // find task with execution time to optimize
    int n = dag_tasks.tasks.size();
    int perfTask = 0;
    std::vector<std::vector<double>> time_limit_options = RecordCloseTimeLimitOptions(dag_tasks);
    for (int i=0;i<time_limit_options.size();i++) {
        if (time_limit_options[i][0]!=-1) {
            perfTask = i;
            break;
        }
    }
    
    OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);

    // -------------------------------------- first time 
    // estimate cpu utilization
    double cpu_util = 0.0;
    int prd0;
    for (int k=0;k<n;k++) {
        if (k==perfTask) {
            const Task t = dag_tasks.GetTask(k);
            prd0 = t.period;            
            continue;
        }
        const Task t = dag_tasks.GetTask(k);
        int prd = t.period;
        GaussianDist g_et = t.getExecGaussian();
        double mu = g_et.mu;
        cpu_util += mu/prd;    
    }
    opt.OptimizeFromScratch_w_TL(n_options);  // result should be 400
    ResourceOptResult res_opt = opt.CollectResults();
    EXPECT_EQ(400, res_opt.id2time_limit[perfTask]);  // SLAM+TSP have high utilization;
    printf("cpu util for other/all tasks: %.2f/%.2f\n",cpu_util,cpu_util+res_opt.id2time_limit[perfTask]/prd0);
    printf("task%d exeT=%.2f\n",perfTask,res_opt.id2time_limit[perfTask]);


    // --------------------------------------- next 
    DAG_Model dag_tasks_updated = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
        "TaskData/test_robotics_v21_2.yaml");  // low utilization     
    for (int k=0;k<n;k++) {
        if (k==perfTask) {        
            double mu = res_opt.id2time_limit[perfTask];
            GaussianDist g = GaussianDist(mu, 0.01);
            FiniteDist eTDist = FiniteDist(g, mu, mu, granuality);
            //printf("set mu=%f\n",mu);
            const_cast<SP_OPT_PA::Task&>(dag_tasks.GetTask(perfTask)).set_execution_time_dist(eTDist);
            const_cast<SP_OPT_PA::Task&>(dag_tasks.GetTask(perfTask)).setExecGaussian(g);  
        } else {
            // for other tasks: update the characteristics
            FiniteDist eTDist = dag_tasks_updated.GetTask(k).execution_time_dist;
            const_cast<SP_OPT_PA::Task&>(dag_tasks.GetTask(k)).set_execution_time_dist(eTDist);
            GaussianDist g = dag_tasks_updated.GetTask(k).getExecGaussian();  
            const_cast<SP_OPT_PA::Task&>(dag_tasks.GetTask(k)).setExecGaussian(g);   
        }
    }

    cpu_util = 0;
    for (int k=0;k<n;k++) {
        if (k==perfTask) {        
            continue;
        }
        const Task t = dag_tasks.GetTask(k);
        int prd = t.period;
        GaussianDist g_et = t.getExecGaussian();
        double mu = g_et.mu;
        cpu_util += mu/prd;    
    }   

    opt.OptimizeIncre_w_TL(dag_tasks, n_options); // call incremental again with updated characteristics    
    res_opt = opt.CollectResults();
    EXPECT_EQ(600,res_opt.id2time_limit[perfTask]);  // should increase a bit
    printf("cpu util for other/all tasks: %.2f/%.2f\n",cpu_util,cpu_util+res_opt.id2time_limit[perfTask]/prd0);
    printf("task%d exeT=%.2f\n",perfTask,res_opt.id2time_limit[perfTask]);

    // --------------------------------------- next 
    //GlobalVariables::debugMode = 1;    
	// read test_robotics_v19_2 again, very high utilization
    dag_tasks_updated = ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
        "TaskData/test_robotics_v19_3.yaml");  // high utilization again 
    for (int k=0;k<n;k++) {
        if (k==perfTask) {        
            double mu = res_opt.id2time_limit[perfTask];
            GaussianDist g = GaussianDist(mu, 0.01);
            FiniteDist eTDist = FiniteDist(g, mu, mu, granuality);
            // printf("set mu=%f\n",mu);
            const_cast<SP_OPT_PA::Task&>(dag_tasks.GetTask(perfTask)).set_execution_time_dist(eTDist);
            const_cast<SP_OPT_PA::Task&>(dag_tasks.GetTask(perfTask)).setExecGaussian(g);  
        } else {
            // for other tasks: update the characteristics
            FiniteDist eTDist = dag_tasks_updated.GetTask(k).execution_time_dist;
            const_cast<SP_OPT_PA::Task&>(dag_tasks.GetTask(k)).set_execution_time_dist(eTDist);
            GaussianDist g = dag_tasks_updated.GetTask(k).getExecGaussian();  
            const_cast<SP_OPT_PA::Task&>(dag_tasks.GetTask(k)).setExecGaussian(g);   
        }
    }

    cpu_util = 0;
    for (int k=0;k<n;k++) {
        if (k==perfTask) {        
            continue;
        }
        const Task t = dag_tasks.GetTask(k);
        int prd = t.period;
        GaussianDist g_et = t.getExecGaussian();
        double mu = g_et.mu;
        cpu_util += mu/prd;    
    }   

    opt.OptimizeIncre_w_TL(dag_tasks, n_options); // call incremental again with updated characteristics    
    res_opt = opt.CollectResults();
    EXPECT_EQ(400,res_opt.id2time_limit[perfTask]);  // should decrease????
    printf("cpu util for other/all tasks: %.2f/%.2f\n",cpu_util,cpu_util+res_opt.id2time_limit[perfTask]/prd0);
    printf("task%d exeT=%.2f\n",perfTask,res_opt.id2time_limit[perfTask]);
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



// read new tasks, and update:
// execution_time_dist, exec_time_gauss
// inline void setExecGaussian(const GaussianDist& exec_gau) {
// inline GaussianDist getExecGaussian() const { return exec_time_gauss; }    
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
    int prdPerf;
    for ( int i=0;i<4;i++ ){
        const Task t = dag_tasks.GetTask(i);
        int prd = t.period;
        if (i==task_wPerf) {
            prdPerf = prd;
            continue;
        }    
        GaussianDist g_et = t.getExecGaussian();
        double mu = g_et.mu;
        cpu_util += mu/prd;
    }
    // print cpu_util, and id2time_limit for task 3
    printf("%02d: cpu_util for other/all=%.4f/%.4ff, exe_time_for_task_%d=%.4f\n",0, 
        cpu_util,cpu_util+res_opt.id2time_limit[task_wPerf]/prdPerf,
        task_wPerf,res_opt.id2time_limit[task_wPerf]); 

    // check k = 1 .. 7 against brute-force scheduler
    double br_rst[] = {-1,90.0,80.556,90.0,80.556,80.556,80.556,80.556};
    // for the rest 30*10s, print cpu_util and res_opt.id2time_limit[task_wPerf]
    // we expect res_opt.id2time_limit[task_wPerf] increase/descrease while cpu_util descrease/increase
    for (int k=1;k<=30;k++) {        
        std::string file_name = "taskset_characteristics_" + std::to_string(k);
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/taskset_cfg_4_5_gen_1/" + file_name + ".yaml";
        DAG_Model dag_tasks_updated = ReadDAG_Tasks(path);

        // update execution_time_dist for task_wPerf
        double mu = res_opt.id2time_limit[task_wPerf];
        GaussianDist g = GaussianDist(mu, 0.01);
        FiniteDist eTDist = FiniteDist(g, mu, mu, 1);
        const_cast<SP_OPT_PA::Task&>(dag_tasks_updated.GetTask(task_wPerf)).set_execution_time_dist(eTDist);
        const_cast<SP_OPT_PA::Task&>(dag_tasks_updated.GetTask(task_wPerf)).setExecGaussian(g);  

        opt.OptimizeIncre_w_TL(dag_tasks_updated, n);
        res_opt = opt.CollectResults();
        
        cpu_util = 0.0;
        for ( int i=0;i<4;i++ ){
            if (i==task_wPerf) continue;
            const Task t = dag_tasks.GetTask(i);
            int prd = t.period;
            GaussianDist g_et = t.getExecGaussian();
            double mu = g_et.mu;
            cpu_util += mu/prd;
        }        
        printf("%02d: cpu_util for other/all=%.4f/%.4ff, exe_time_for_task_%d=%.4f\n",k, 
            cpu_util,cpu_util+res_opt.id2time_limit[task_wPerf]/prdPerf,
            task_wPerf,res_opt.id2time_limit[task_wPerf]); 
        if (k>=1 && k<=7) {
           double v = abs(res_opt.id2time_limit[task_wPerf] - br_rst[k]);
           EXPECT_LT(v, 0.001);  // hopefully result == brute-force result
        }
    }
}


int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}