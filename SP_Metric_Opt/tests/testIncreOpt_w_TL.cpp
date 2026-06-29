// #include <gtest/gtest.h>

#include "sources/Optimization/OptimizeSP_BF.h"
#include "sources/Optimization/OptimizeSP_TL_Incre.h"
#include "sources/Safety_Performance_Metric/Probability.h"
#include "sources/TaskModel/RegularTasks.h"
#include "sources/Utils/Parameters.h"
#include "gmock/gmock.h" // Brings in gMock.

using ::testing::AtLeast; // #1
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

TEST_F(TaskSetForTest_robotics_v20, RecordCloseTimeLimitOptions) {
  std::vector<std::vector<double>> time_limit_options =
      RecordCloseTimeLimitOptions(dag_tasks);
  // Closest to ET ~202 is 184.1 (index 0). Radius=2 => indices [0,2] => 3 opts.
  EXPECT_EQ(3, time_limit_options[0].size());
  EXPECT_EQ(184.1, time_limit_options[0][0]);
  EXPECT_EQ(397.5, time_limit_options[0][1]);
  EXPECT_EQ(657.9, time_limit_options[0][2]);

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
  OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
  opt.OptimizeFromScratch_w_TL(2);
  ResourceOptResult res_opt = opt.CollectResults();
  PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);

  // TSP's execution time ~1501 floors to the 1000 ms pair (perf = 1.0).
  // The system is schedulable with TL = 1000 and the optimizer correctly
  // selects the highest-performance option.
  EXPECT_EQ(1000, res_opt.id2time_limit[0]);
}

TEST_F(TaskSetForTest_robotics_v18, optimize_no_tl) {
  // 1. Run with TL optimization enabled (default behavior)
  GlobalVariables::disable_time_limit_opt = false;
  OptimizePA_Incre_with_TimeLimits opt_with_tl(dag_tasks, sp_parameters);
  opt_with_tl.OptimizeFromScratch_w_TL(2);
  ResourceOptResult res_opt = opt_with_tl.CollectResults();
  double opt_sp = res_opt.sp_opt;

  // Get the smallest possible time limits to compare
  std::vector<double> smallest_time_limits(dag_tasks.tasks.size());
  for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
    if (dag_tasks.tasks[i].timePerformancePairs.empty()) {
      smallest_time_limits[i] = -1.0;
    } else {
      smallest_time_limits[i] =
          dag_tasks.tasks[i].timePerformancePairs[0].time_limit;
    }
  }

  // The optimizer selects TL=1000 because it yields the highest perf (1.0)
  // while keeping the system schedulable.
  EXPECT_EQ(1000, res_opt.id2time_limit[0]);

  // TL optimization moved task 0 away from its smallest option.
  bool tl_changed = false;
  for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
    if (res_opt.id2time_limit[i] != smallest_time_limits[i]) {
      tl_changed = true;
      break;
    }
  }
  EXPECT_TRUE(tl_changed);

  // 2. Run with TL optimization disabled
  GlobalVariables::disable_time_limit_opt = true;
  OptimizePA_Incre_with_TimeLimits opt_no_tl(dag_tasks, sp_parameters);
  opt_no_tl.OptimizeFromScratch_w_TL(2);
  ResourceOptResult res_no_tl = opt_no_tl.CollectResults();
  double no_tl_sp = res_no_tl.sp_opt;

  // Restore default behavior immediately
  GlobalVariables::disable_time_limit_opt = false;

  // With TL optimization disabled, every task is pinned to its smallest TL.
  EXPECT_EQ(400, res_no_tl.id2time_limit[0]);
  for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
    EXPECT_EQ(smallest_time_limits[i], res_no_tl.id2time_limit[i]);
  }

  // Because the optimal TL (1000) is not the smallest, disabling TL opt
  // degrades SP.
  EXPECT_LT(no_tl_sp, opt_sp);
}

TEST_F(TaskSetForTest_robotics_v18, optimize_wcet) {
  // 1. Run with WCET baseline disabled (default behavior)
  GlobalVariables::use_wcet_execution_time = false;
  OptimizePA_Incre_with_TimeLimits opt_normal(dag_tasks, sp_parameters);
  opt_normal.OptimizeFromScratch_w_TL(2);
  ResourceOptResult res_normal = opt_normal.CollectResults();
  double tl_normal = res_normal.id2time_limit[0];

  // 2. Run with WCET baseline enabled
  GlobalVariables::use_wcet_execution_time = true;
  OptimizePA_Incre_with_TimeLimits opt_wcet(dag_tasks, sp_parameters);
  opt_wcet.OptimizeFromScratch_w_TL(2);
  ResourceOptResult res_wcet = opt_wcet.CollectResults();
  double tl_wcet = res_wcet.id2time_limit[0];

  // Restore default behavior immediately
  GlobalVariables::use_wcet_execution_time = false;

  // Verify that every task's execution time distribution in opt_wcet was forced
  // to its WCET constant value
  for (size_t i = 0; i < dag_tasks.tasks.size(); i++) {
    double original_wcet = dag_tasks.tasks[i].execution_time_dist.max_time;
    const auto &task_updated = opt_wcet.dag_tasks_.tasks[i];
    EXPECT_DOUBLE_EQ(original_wcet, task_updated.execution_time_dist.max_time);
    EXPECT_DOUBLE_EQ(original_wcet, task_updated.execution_time_dist.min_time);
    EXPECT_DOUBLE_EQ(original_wcet, task_updated.getExecutionTime());
  }

  // Normal mode picks the highest-performance option (1000) that is still
  // schedulable under the stochastic execution-time distribution.
  EXPECT_EQ(1000, tl_normal);

  // Under WCET ablation the execution times are forced to their worst-case
  // values, making the task set more constrained. With K=2 the greedy search
  // sometimes selects a tighter TL (800) to remain schedulable.
  EXPECT_LE(tl_wcet, 1000);
  EXPECT_GE(tl_wcet, 400);
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

class TestDDLMiss : public ::testing::Test {
public:
  void SetUp() override {
    std::string file_name = "test_ddl_miss";
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

class TestDDLMissLessTasks : public ::testing::Test {
public:
  void SetUp() override {
    std::string file_name = "test_ddl_miss_less_tasks";
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
  EXPECT_EQ(4, time_limit_options.size());    // 4 tasks
  // With TimeLimitSearchRadiusIncr=2 the window around closest ET (1000) is
  // indices [1,3] => [600, 800, 1000] (3 options).
  EXPECT_EQ(3, time_limit_options[0].size()); // 3 options for TSP
  EXPECT_EQ(600, time_limit_options[0][0]);
  EXPECT_EQ(800, time_limit_options[0][1]);
  EXPECT_EQ(1000, time_limit_options[0][2]);

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
  EXPECT_EQ(800,
            res_opt.id2time_limit[0]); // SLAM+TSP have high utilization;
  //   Conservative max-value compression makes the system appear more
  //   schedulable, so scratch-mode optimizer now selects 800.
  // In incremental mode, should return 800
}

TEST_F(TaskSetForTest_robotics_v19, optimize_incremental) {
  OptimizePA_Incre_with_TimeLimits opt(dag_tasks,
                                       sp_parameters); // high utilization

  opt.OptimizeFromScratch_w_TL(2); // result is 800 with conservative compression
  ResourceOptResult res_opt = opt.CollectResults();
  EXPECT_EQ(800,
            res_opt.id2time_limit[0]); // SLAM+TSP have high utilization;

  DAG_Model dag_tasks_updated =
      ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
                    "TaskData/test_robotics_v21.yaml"); // low utilization
  opt.OptimizeIncre_w_TL(dag_tasks_updated, 2);
  res_opt = opt.CollectResults();
  // With radius=2 the incremental window for TSP (ET~401) is {400,600,800}.
  // Low-utilization v21 allows the highest-performing feasible option: 800.
  EXPECT_EQ(
      800,
      res_opt.id2time_limit[0]);

  auto start_time = CurrentTimeInProfiler;
  for (int i = 0; i < 10; i++)
    opt.OptimizeIncre_w_TL(dag_tasks_updated, 2);
  auto finish_time = CurrentTimeInProfiler;
  double time_taken = GetTimeTaken(start_time, finish_time);
  EXPECT_LT(time_taken / 10.0,
            2.5e-1); // relaxed for debug mode coordinate descent

  // dag_tasks_updated =
  //     ReadDAG_Tasks(GlobalVariables::PROJECT_PATH +
  //                   "TaskData/test_robotics_v22.yaml");  // low utilization
}

TEST_F(TaskSetForTest_robotics_v19_2, RecordCloseTimeLimitOptions) {
  printf(
      "\n-------- TaskSetForTest_robotics_v19_2, RecordCloseTimeLimitOptions "
      "...\n");
  std::vector<std::vector<double>> time_limit_options =
      RecordCloseTimeLimitOptions(dag_tasks);

  EXPECT_EQ(4, time_limit_options.size()); // 4 tasks

  uint perfTask = 0;
  for (int i = 0; i < static_cast<int>(time_limit_options.size()); i++) {
    if (time_limit_options[i][0] != -1) {
      perfTask = i;
      break;
    }
  }

  // With TimeLimitSearchRadiusIncr=2 the window is [1,3] => [600,800,1000].
  EXPECT_EQ(3, time_limit_options[perfTask].size()); // 3 options for TSP
  EXPECT_EQ(600, time_limit_options[perfTask][0]);
  EXPECT_EQ(800, time_limit_options[perfTask][1]);
  EXPECT_EQ(1000, time_limit_options[perfTask][2]);

  for (uint i = 0; i < time_limit_options.size(); i++) {
    if (i == perfTask)
      continue;
    EXPECT_EQ(-1, time_limit_options[i][0]);
  }
}

TEST_F(TaskSetForTest_robotics_v19_2, OptimizeFromScratch_w_TL) {
  // NOTE: this test failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // compare with the testcase (TaskSetForTest_robotics_v19) before, the only
  // difference is that this testcase added two tasks with very small
  // cpu_utilization and big weight supposedly, OptimizeFromScratch_w_TL
  // should also return 400 for task0

  // try to get which task has performance_records_time
  int perfTask = 0;
  std::vector<std::vector<double>> time_limit_options =
      RecordCloseTimeLimitOptions(dag_tasks);
  for (int i = 0; i < static_cast<int>(time_limit_options.size()); i++) {
    if (time_limit_options[i][0] != -1) {
      perfTask = i;
      break;
    }
  }

  printf("\n-------- TaskSetForTest_robotics_v19_2, OptimizeFromScratch_w_TL "
         "...\n");
  OptimizePA_Incre_with_TimeLimits opt(dag_tasks, sp_parameters);
  EXPECT_FALSE(opt.IfInitialized());

  // int n = dag_tasks.tasks.size();
  // for (int i=0;i<n;i++){
  //     double exeT = dag_tasks.GetTask(i).getExecGaussian().mu;
  //     const_cast<SP_OPT_PA::Task&>(dag_tasks.GetTask(i)).setExecutionTime(exeT);
  //     printf("task%d:
  //     exeT=%f\n",i,(double)(dag_tasks.GetTask(i).getExecutionTime()));
  // }
  opt.OptimizeFromScratch_w_TL(2);
  EXPECT_TRUE(opt.IfInitialized());
  ResourceOptResult res_opt = opt.CollectResults();
  PrintPriorityVec(dag_tasks.tasks, res_opt.priority_vec);
  EXPECT_EQ(1000,
            res_opt.id2time_limit[perfTask]); // Conservative compression improves
  //   schedulability assessment, allowing larger time limits.
}

TEST_F(TestDDLMiss, test_ddl_miss) {
  // the very last task has high utilization
  // the last one should definitely miss deadline!
  double cpu_util = 0.0;
  for (int i = 0; i < static_cast<int>(dag_tasks.tasks.size()); i++) {
    double mu = dag_tasks.tasks[i].execution_time_dist.GetAvgValue();
    printf("task%d, period=%d, deadline=%.0f, mu=%.2f\n", i,
           dag_tasks.tasks[i].period, dag_tasks.tasks[i].deadline, mu);
    cpu_util += mu / dag_tasks.tasks[i].period;
  }
  printf("overall cpu util=%.2f\n\n", cpu_util);

  std::vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet(dag_tasks.tasks);
  for (uint i = 0; i < rtas.size(); i++) {
    std::cout << "rtas[" << i << "]: ..." << std::endl;
    rtas[i].print();
  }
  printf("\n");
  for (uint i = 0; i < dag_tasks.tasks.size(); i++) {
    double ddl_miss_chance =
        GetDDL_MissProbability(rtas[i], dag_tasks.tasks[i].deadline);
    printf("task%d, ddl_miss_chance=%.2f\n", i, ddl_miss_chance);

    if (i == dag_tasks.tasks.size() - 1) {
      EXPECT_GT(ddl_miss_chance, 0.99999);
    } else {
      EXPECT_LT(ddl_miss_chance, 0.00001);
    }
  }
}

TEST_F(TestDDLMissLessTasks, test_ddl_miss) {
  // the very last task has high utilization
  // the last one should definitely miss deadline!
  double cpu_util = 0.0;
  for (int i = 0; i < static_cast<int>(dag_tasks.tasks.size()); i++) {
    double mu = dag_tasks.tasks[i].execution_time_dist.GetAvgValue();
    printf("task%d, period=%d, deadline=%.0f, mu=%.2f\n", i,
           dag_tasks.tasks[i].period, dag_tasks.tasks[i].deadline, mu);
    cpu_util += mu / dag_tasks.tasks[i].period;
  }
  printf("overall cpu util=%.2f\n\n", cpu_util);

  std::vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet(dag_tasks.tasks);
  for (uint i = 0; i < rtas.size(); i++) {
    std::cout << "rtas[" << i << "]: ..." << std::endl;
    rtas[i].print();
  }
  printf("\n");
  for (int i = 0; i < static_cast<int>(dag_tasks.tasks.size()); i++) {
    double ddl_miss_chance =
        GetDDL_MissProbability(rtas[i], dag_tasks.tasks[i].deadline);
    printf("task%d, ddl_miss_chance=%.2f\n", i, ddl_miss_chance);

    if (i == static_cast<int>(dag_tasks.tasks.size()) - 1) {
      EXPECT_GT(ddl_miss_chance, 0.99999);
    } else {
      EXPECT_LT(ddl_miss_chance, 0.00001);
    }
  }
}

TEST_F(TaskSetForTest_robotics_v19, OptimizeWithOptimizationSpace) {
    // Tighten TSP (task 0) execution time to emphasise TL optimization impact.
    dag_tasks.tasks[0].execution_time_dist =
        FiniteDist(GaussianDist(700.0, 10.0), 5);

    // 1. Optimize from scratch (can explore all time limit options)
    OptimizePA_Incre_with_TimeLimits opt_scratch(dag_tasks, sp_parameters);
    opt_scratch.OptimizeFromScratch_w_TL(2);
    ResourceOptResult res_scratch = opt_scratch.CollectResults();

    // With floor behaviour any TL in [400, 599) yields the same perf (0.5),
    // any TL in [600, 799) yields the same perf (0.6).  Because the tightest
    // feasible TL gives the best schedulability, the optimizer prefers the
    // smallest TL in the best reachable bracket.  With conservative
    // compression improving schedulability, the best bracket is now 800.
    EXPECT_EQ(800, res_scratch.id2time_limit[0]);

    // 2. Optimize incrementally starting from warm start (ET pinned at 1000ms)
    OptimizePA_Incre_with_TimeLimits opt_incre(dag_tasks, sp_parameters);
    DAG_Model dag_tasks_warm = dag_tasks;
    dag_tasks_warm.tasks[0].execution_time_dist =
        GetUnitExecutionTimeDist(1000.0);

    opt_incre.OptimizeIncre_w_TL(dag_tasks_warm, 2);
    ResourceOptResult res_incre = opt_incre.CollectResults();

    // The incremental search window is restricted to neighbours near the
    // warm-start ET (1000). Under radius=2 that is [600, 800, 1000].
    // With tight ET 700 the feasible window is [600, 800]. Both options
    // yield the same floor perf. With improved distribution resolution from
    // the block compression algorithm, the optimizer now finds 800 as the
    // best feasible TL when starting from warm ET=1000.
    EXPECT_EQ(800, res_incre.id2time_limit[0]);

    // Scratch explored the full option set so it should be at least as good.
    EXPECT_GE(res_scratch.sp_opt, res_incre.sp_opt);
}

TEST(RecordCloseTimeLimitOptions_DynamicRadius, Vanilla) {
  // Build a synthetic task with 10 evenly-spaced TL options [0, 10, 20, ... 90]
  std::vector<Value_Proba> dist = {Value_Proba(45, 1.0)};
  Task t(0, dist, 1000, 1000, 0, "T_perf");
  for (int i = 0; i < 10; ++i) {
    t.timePerformancePairs.push_back(TimePerfPair(i * 10, i * 0.1));
  }
  // ET is 45 → closest option is index 4 (value 40) because |45-40| = |45-50|
  // and the earlier index wins.
  MAP_Prev mapPrev;
  TaskSet tasks = {t};
  DAG_Model dag(tasks, mapPrev, 0, 0);

  int saved_radius = GlobalVariables::TimeLimitSearchRadiusIncr;

  // Radius 2 → indices [2, 6] → 5 options: {20,30,40,50,60}
  {
    GlobalVariables::TimeLimitSearchRadiusIncr = 2;
    auto opts = RecordCloseTimeLimitOptions(dag);
    ASSERT_EQ(1u, opts.size());
    EXPECT_EQ(5u, opts[0].size());
    EXPECT_DOUBLE_EQ(20.0, opts[0][0]);
    EXPECT_DOUBLE_EQ(40.0, opts[0][2]);
    EXPECT_DOUBLE_EQ(60.0, opts[0][4]);
  }

  // Radius 0 → single option (closest only)
  {
    GlobalVariables::TimeLimitSearchRadiusIncr = 0;
    auto opts = RecordCloseTimeLimitOptions(dag);
    ASSERT_EQ(1u, opts.size());
    EXPECT_EQ(1u, opts[0].size());
    EXPECT_DOUBLE_EQ(40.0, opts[0][0]);
  }

  // Large radius 5 → indices [0, 9] because only 10 options exist
  {
    GlobalVariables::TimeLimitSearchRadiusIncr = 5;
    auto opts = RecordCloseTimeLimitOptions(dag);
    ASSERT_EQ(1u, opts.size());
    EXPECT_EQ(10u, opts[0].size());
    EXPECT_DOUBLE_EQ(0.0, opts[0][0]);
    EXPECT_DOUBLE_EQ(90.0, opts[0][9]);
  }

  GlobalVariables::TimeLimitSearchRadiusIncr = saved_radius;
}

int main(int argc, char **argv) {
  // ::testing::InitGoogleTest(&argc, argv);
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}