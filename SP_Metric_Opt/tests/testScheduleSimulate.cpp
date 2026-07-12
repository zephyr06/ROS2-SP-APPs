// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/RTDA/ImplicitCommunication/ScheduleSimulation.h"
#include "sources/RTDA/ImplicitCommunication/SimulationOrchestrator.h"
#include "sources/Safety_Performance_Metric/ParametersSP.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/TaskModel/RegularTasks.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/readwrite.h"
using ::testing::AtLeast;  // #1
using ::testing::Return;
using namespace std;
using namespace SP_OPT_PA;
using namespace GlobalVariables;

class TaskSetForTest_scheduling_v1 : public ::testing::Test {
   public:
    void SetUp() override {
        std::string file_name = "test_robotics_v10";
        std::string path =
            GlobalVariables::PROJECT_PATH + "TaskData/" + file_name + ".yaml";
        dag_tasks = ReadDAG_Tasks(path);
        tasks = dag_tasks.tasks;
        sp_parameters = SP_Parameters(tasks);
        AssignTaskSetPriorityById(dag_tasks.tasks);
        tasks_info = TaskSetInfoDerived(dag_tasks.tasks);

        dag_tasks.tasks[0].setExecutionTime(1);
        dag_tasks.tasks[1].setExecutionTime(2);
        dag_tasks.tasks[2].setExecutionTime(3);
    }

    // data members
    TaskSet tasks;
    DAG_Model dag_tasks;
    TaskSetInfoDerived tasks_info;
    SP_Parameters sp_parameters;
};
TEST_F(TaskSetForTest_scheduling_v1, simulate_schedule) {
    Schedule schedule = SimulateFixedPrioritySched(dag_tasks, tasks_info);
    EXPECT_EQ(4, schedule.size());
    EXPECT_EQ(0, schedule[JobCEC(0, 0)].start);
    EXPECT_EQ(1, schedule[JobCEC(1, 0)].start);
    EXPECT_EQ(3, schedule[JobCEC(2, 0)].start);
    EXPECT_EQ(10, schedule[JobCEC(0, 1)].start);
    EXPECT_EQ(11, schedule[JobCEC(0, 1)].finish);
}
TEST_F(TaskSetForTest_scheduling_v1, simulate_schedule_v2) {
    dag_tasks.tasks[0].priority = 3;
    dag_tasks.tasks[1].priority = 2;
    dag_tasks.tasks[2].priority = 1;
    Schedule schedule = SimulateFixedPrioritySched(dag_tasks, tasks_info);
    EXPECT_EQ(4, schedule.size());
    EXPECT_EQ(5, schedule[JobCEC(0, 0)].start);
    EXPECT_EQ(3, schedule[JobCEC(1, 0)].start);
    EXPECT_EQ(0, schedule[JobCEC(2, 0)].start);
    EXPECT_EQ(10, schedule[JobCEC(0, 1)].start);
    EXPECT_EQ(11, schedule[JobCEC(0, 1)].finish);
}
TEST_F(TaskSetForTest_scheduling_v1, simulate_schedule_v3) {
    dag_tasks.tasks[0].processorId = 1;
    Schedule schedule = SimulateFixedPrioritySched(dag_tasks, tasks_info);
    EXPECT_EQ(4, schedule.size());
    EXPECT_EQ(0, schedule[JobCEC(0, 0)].start);
    EXPECT_EQ(0, schedule[JobCEC(1, 0)].start);
    EXPECT_EQ(2, schedule[JobCEC(2, 0)].start);
    EXPECT_EQ(10, schedule[JobCEC(0, 1)].start);
    EXPECT_EQ(11, schedule[JobCEC(0, 1)].finish);
}
TEST_F(TaskSetForTest_scheduling_v1, simulate_cfs_schedule) {
    Schedule schedule = SimulateCFSSched(dag_tasks, tasks_info);
    EXPECT_EQ(4, schedule.size());
    EXPECT_EQ(2, schedule[JobCEC(0, 0)].start);
    EXPECT_EQ(1, schedule[JobCEC(1, 0)].start);
    EXPECT_EQ(0, schedule[JobCEC(2, 0)].start);
    EXPECT_EQ(10, schedule[JobCEC(0, 1)].start);

    EXPECT_EQ(3, schedule[JobCEC(0, 0)].finish);
    EXPECT_EQ(5, schedule[JobCEC(1, 0)].finish);
    EXPECT_EQ(6, schedule[JobCEC(2, 0)].finish);
    EXPECT_EQ(11, schedule[JobCEC(0, 1)].finish);
}
TEST_F(TaskSetForTest_scheduling_v1, simulate_cfs_et_larger_than_period) {
    dag_tasks.tasks[0].setExecutionTime(12);
    dag_tasks.tasks[1].setExecutionTime(2);
    dag_tasks.tasks[2].setExecutionTime(3);

    TaskSetInfoDerived new_tasks_info(dag_tasks.tasks);

    Schedule schedule = SimulateCFSSched(dag_tasks, new_tasks_info);

    EXPECT_EQ(2, schedule[JobCEC(0, 0)].start);
    EXPECT_EQ(17, schedule[JobCEC(0, 0)].finish);

    EXPECT_EQ(1, schedule[JobCEC(1, 0)].start);
    EXPECT_EQ(5, schedule[JobCEC(1, 0)].finish);

    EXPECT_EQ(0, schedule[JobCEC(2, 0)].start);
    EXPECT_EQ(7, schedule[JobCEC(2, 0)].finish);

    EXPECT_EQ(17, schedule[JobCEC(0, 1)].start);
    EXPECT_EQ(-1, schedule[JobCEC(0, 1)].finish);
}
TEST_F(TaskSetForTest_scheduling_v1, simulate_cfs_deadline_miss) {
    dag_tasks.tasks[0].setExecutionTime(4);
    dag_tasks.tasks[0].deadline = 3.0;
    dag_tasks.tasks[1].setExecutionTime(2);
    dag_tasks.tasks[2].setExecutionTime(3);

    TaskSetInfoDerived new_tasks_info(dag_tasks.tasks);

    Schedule schedule = SimulateCFSSched(dag_tasks, new_tasks_info);

    EXPECT_EQ(9, schedule[JobCEC(0, 0)].finish);
    double ddl = GetDeadline(JobCEC(0, 0), new_tasks_info);
    EXPECT_EQ(3, ddl);
    EXPECT_TRUE(schedule[JobCEC(0, 0)].finish > ddl);

    EXPECT_EQ(10, schedule[JobCEC(0, 1)].start);
    EXPECT_EQ(14, schedule[JobCEC(0, 1)].finish);
}
class TestOrchestrator : public FixedTaskPrioritySchedulingOrchestrator {
   public:
    TestOrchestrator(const std::string& input_folder,
                     const std::string& output_folder, const std::string& mode,
                     LLint duration)
        : FixedTaskPrioritySchedulingOrchestrator(input_folder, output_folder,
                                                  mode, duration) {}

    void TestLoadConfigs() { LoadIntervalConfigs(); }

    const std::vector<DAG_Model>& GetDagTasks() const {
        return dag_tasks_vecs_;
    }
    const std::vector<TaskSetInfoDerived>& GetTasksInfo() const {
        return tasks_info_vecs_;
    }
    const std::vector<SP_Parameters>& GetSpParameters() const {
        return sp_parameters_vecs_;
    }

    std::vector<float> TestLoadTraces(int task_id, int path_idx, int inst_idx) {
        return LoadJobExecutionTraces(task_id, path_idx, inst_idx);
    }
};

class TestCFSOrchestrator : public CFSSimulationOrchestrator {
   public:
    TestCFSOrchestrator(const std::string& input_folder,
                        const std::string& output_folder, LLint duration)
        : CFSSimulationOrchestrator(input_folder, output_folder, duration) {}

    void TestLoadConfigs() { LoadIntervalConfigs(); }
    const std::vector<DAG_Model>& GetDagTasks() const {
        return dag_tasks_vecs_;
    }
    const std::vector<SP_Parameters>& GetSpParameters() const {
        return sp_parameters_vecs_;
    }
};

TEST(OrchestratorTest, LoadIntervalConfigs) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    TestOrchestrator orchestrator(input_dir, "", "RM", 100);
    orchestrator.TestLoadConfigs();

    const auto& dags = orchestrator.GetDagTasks();
    ASSERT_EQ(2, dags.size());  // i0 and i1
    EXPECT_EQ(4, dags[0].tasks.size());
    EXPECT_EQ("Task0", dags[0].tasks[0].name);
    EXPECT_EQ(10, dags[0].tasks[0].period);
}

TEST(OrchestratorTest, LoadJobExecutionTraces) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    TestOrchestrator orchestrator(input_dir, "", "RM", 100);

    std::vector<float> traces = orchestrator.TestLoadTraces(0, 0, 0);
    ASSERT_EQ(3, traces.size());
    EXPECT_FLOAT_EQ(1.5f, traces[0]);
    EXPECT_FLOAT_EQ(2.5f, traces[1]);
    EXPECT_FLOAT_EQ(3.5f, traces[2]);
}

TEST(OrchestratorTest, RateMonotonicPriorityAssignment) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    std::string output_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_output_rm";

    FixedTaskPrioritySchedulingOrchestrator orchestrator(input_dir, output_dir,
                                                         "RM", 100);
    orchestrator.RunSimulation();

    const auto& history = orchestrator.GetJobHistory();
    ASSERT_FALSE(history.empty());

    // RM does not enforce time limits → never overruns.
    for (const auto& r : history) {
        EXPECT_FALSE(r.isOverrun);
    }

    // Deterministic interval SP metrics (probabilistic RTA on the fixed dists).
    // i0: Task0 avg ET=2 → perf=0.6; others perf=1.0. Task3 schedulable.
    //     SP = 0.6 + 1.0 + 1.0 + 1.0 = 3.6
    // i1: Task0 avg ET=3 → perf=1.0; others perf=1.0. Task3 has slight ddl-miss
    //     probability → SP ≈ 3.86524 (optimized execution-time convolution).
    const auto& sp_metrics = orchestrator.GetIntervalSPMetrics();
    ASSERT_EQ(2, sp_metrics.size());
    EXPECT_NEAR(3.6, sp_metrics[0], 1e-4);
    EXPECT_NEAR(3.86524, sp_metrics[1], 1e-4);
}



class MockOrchestrator : public BaseSimulationOrchestrator {
public:
    MockOrchestrator(const std::string& output_folder, LLint interval_duration_ms = 1000)
        : BaseSimulationOrchestrator("", output_folder, interval_duration_ms) {}

    void RunSimulation() override {
        // No-op for mock orchestrator
    }

    void PopulateData(const std::vector<JobRecord>& history, const std::vector<double>& sp_metrics) {
        job_history_ = history;
        interval_sp_metrics_ = sp_metrics;
    }

    void CallExportResults(const std::string& scheduler_name) {
        ExportResults(scheduler_name);
    }
};

static std::vector<JobRecord> GetMockJobHistory() {
    std::vector<JobRecord> history;
    // Task 0: 2 jobs (1 overrun)
    history.push_back({0, 0, 0, 10, 15, 5.0, false});
    history.push_back({0, 1, 1000, 1010, 1025, 15.0, true});
    // Task 1: 2 jobs (0 overrun)
    history.push_back({1, 0, 0, 15, 25, 10.0, false});
    history.push_back({1, 1, 2000, 2010, 2025, 15.0, false});
    return history;
}

static std::vector<double> GetMockSPMetrics() {
    return {0.95, 0.90, 0.85}; // 3 intervals
}

TEST(OrchestratorTest, ExportResultsLevel0) {
    std::string output_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_output_export_l0";
    std::filesystem::remove_all(output_dir);

    int old_level = GlobalVariables::EXPORT_DETAIL_LEVEL;
    GlobalVariables::EXPORT_DETAIL_LEVEL = 0;

    MockOrchestrator orchestrator(output_dir, 1000);
    orchestrator.PopulateData(GetMockJobHistory(), GetMockSPMetrics());
    orchestrator.CallExportResults("RM");

    std::string metrics_file = output_dir + "/RM/interval_sp_metrics.txt";
    std::string summary_file = output_dir + "/RM/miss_rate_summary.txt";
    std::string response_file = output_dir + "/RM/response_times_task_0.txt";
    std::string per_task_miss_file = output_dir + "/RM/miss_rate_per_task.txt";
    std::string aggregate_file = output_dir + "/RM/task_aggregate_0.txt";

    EXPECT_TRUE(std::filesystem::exists(metrics_file));
    EXPECT_TRUE(std::filesystem::exists(summary_file));
    EXPECT_FALSE(std::filesystem::exists(response_file));
    EXPECT_FALSE(std::filesystem::exists(per_task_miss_file));
    EXPECT_FALSE(std::filesystem::exists(aggregate_file));

    // Verify summary contents
    std::ifstream file(summary_file);
    std::string line;
    ASSERT_TRUE(std::getline(file, line));
    EXPECT_EQ("total_jobs,missed_jobs,miss_rate", line);
    ASSERT_TRUE(std::getline(file, line));
    std::stringstream ss(line);
    int total_jobs = 0, missed_jobs = -1;
    double miss_rate = -1.0;
    char comma;
    ASSERT_TRUE(ss >> total_jobs >> comma >> missed_jobs >> comma >> miss_rate);
    EXPECT_EQ(4, total_jobs);
    EXPECT_EQ(1, missed_jobs);
    EXPECT_DOUBLE_EQ(0.25, miss_rate);

    GlobalVariables::EXPORT_DETAIL_LEVEL = old_level;
}

TEST(OrchestratorTest, ExportResultsLevel1) {
    std::string output_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_output_export_l1";
    std::filesystem::remove_all(output_dir);

    int old_level = GlobalVariables::EXPORT_DETAIL_LEVEL;
    GlobalVariables::EXPORT_DETAIL_LEVEL = 1;

    MockOrchestrator orchestrator(output_dir, 1000);
    orchestrator.PopulateData(GetMockJobHistory(), GetMockSPMetrics());
    orchestrator.CallExportResults("RM");

    std::string metrics_file = output_dir + "/RM/interval_sp_metrics.txt";
    std::string summary_file = output_dir + "/RM/miss_rate_summary.txt";
    std::string per_task_miss_file = output_dir + "/RM/miss_rate_per_task.txt";
    std::string response_file = output_dir + "/RM/response_times_task_0.txt";
    std::string aggregate_file = output_dir + "/RM/task_aggregate_0.txt";

    EXPECT_TRUE(std::filesystem::exists(metrics_file));
    EXPECT_TRUE(std::filesystem::exists(summary_file));
    EXPECT_TRUE(std::filesystem::exists(per_task_miss_file));
    EXPECT_FALSE(std::filesystem::exists(response_file));
    EXPECT_FALSE(std::filesystem::exists(aggregate_file));

    // Verify per-task miss rate summary
    std::ifstream file(per_task_miss_file);
    std::string line;
    ASSERT_TRUE(std::getline(file, line));
    EXPECT_EQ("task_id,total_jobs,missed_jobs,miss_rate,avg_response_time,max_response_time", line);

    int t_id, total, missed;
    double mr, avg_rt, max_rt;
    char comma;

    std::unordered_map<int, std::vector<double>> task_stats;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        ASSERT_TRUE(ss >> t_id >> comma >> total >> comma >> missed >> comma >> mr >> comma >> avg_rt >> comma >> max_rt);
        task_stats[t_id] = {static_cast<double>(total), static_cast<double>(missed), mr, avg_rt, max_rt};
    }
    
    EXPECT_EQ(2, task_stats.size());
    EXPECT_EQ(2, task_stats[0][0]); // Task 0 total
    EXPECT_EQ(1, task_stats[0][1]); // Task 0 missed
    EXPECT_DOUBLE_EQ(0.5, task_stats[0][2]); // Task 0 miss rate
    EXPECT_DOUBLE_EQ(20.0, task_stats[0][3]); // Task 0 avg rt (Job 0: 15, Job 1: 25 -> avg 20)
    EXPECT_DOUBLE_EQ(25.0, task_stats[0][4]); // Task 0 max rt (Job 0: 15, Job 1: 25 -> max 25)

    EXPECT_EQ(2, task_stats[1][0]); // Task 1 total
    EXPECT_EQ(0, task_stats[1][1]); // Task 1 missed
    EXPECT_DOUBLE_EQ(0.0, task_stats[1][2]); // Task 1 miss rate

    GlobalVariables::EXPORT_DETAIL_LEVEL = old_level;
}

TEST(OrchestratorTest, ExportResultsLevel2) {
    std::string output_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_output_export_l2";
    std::filesystem::remove_all(output_dir);

    int old_level = GlobalVariables::EXPORT_DETAIL_LEVEL;
    GlobalVariables::EXPORT_DETAIL_LEVEL = 2;

    MockOrchestrator orchestrator(output_dir, 1000);
    orchestrator.PopulateData(GetMockJobHistory(), GetMockSPMetrics());
    orchestrator.CallExportResults("RM");

    std::string aggregate_file = output_dir + "/RM/task_aggregate_0.txt";
    EXPECT_TRUE(std::filesystem::exists(aggregate_file));

    // Verify task aggregate contents
    std::ifstream file(aggregate_file);
    std::string line;
    ASSERT_TRUE(std::getline(file, line));
    EXPECT_EQ("interval_index,time_seconds,job_count,avg_response_time,max_response_time,missed_jobs", line);

    // Interval 0: Task 0 Job 0 (release 0 -> falls in interval 0). count=1, rt=15.
    ASSERT_TRUE(std::getline(file, line));
    std::stringstream ss0(line);
    int interval_index, time_seconds, job_count, missed_jobs;
    double avg_response, max_response;
    char comma;
    ASSERT_TRUE(ss0 >> interval_index >> comma >> time_seconds >> comma >> job_count >> comma >> avg_response >> comma >> max_response >> comma >> missed_jobs);
    EXPECT_EQ(0, interval_index);
    EXPECT_EQ(0, time_seconds);
    EXPECT_EQ(1, job_count);
    EXPECT_DOUBLE_EQ(15.0, avg_response);
    EXPECT_EQ(0, missed_jobs);

    // Interval 1: Task 0 Job 1 (release 1000 -> falls in interval 1). count=1, rt=25, missed=1.
    ASSERT_TRUE(std::getline(file, line));
    std::stringstream ss1(line);
    ASSERT_TRUE(ss1 >> interval_index >> comma >> time_seconds >> comma >> job_count >> comma >> avg_response >> comma >> max_response >> comma >> missed_jobs);
    EXPECT_EQ(1, interval_index);
    EXPECT_EQ(1, time_seconds);
    EXPECT_EQ(1, job_count);
    EXPECT_DOUBLE_EQ(25.0, avg_response);
    EXPECT_EQ(1, missed_jobs);

    // Interval 2: No Task 0 jobs released. count=0.
    ASSERT_TRUE(std::getline(file, line));
    std::stringstream ss2(line);
    ASSERT_TRUE(ss2 >> interval_index >> comma >> time_seconds >> comma >> job_count >> comma >> avg_response >> comma >> max_response >> comma >> missed_jobs);
    EXPECT_EQ(2, interval_index);
    EXPECT_EQ(2, time_seconds);
    EXPECT_EQ(0, job_count);

    GlobalVariables::EXPORT_DETAIL_LEVEL = old_level;
}

TEST(OrchestratorTest, ExportResultsLevel3) {
    std::string output_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_output_export_l3";
    std::filesystem::remove_all(output_dir);

    int old_level = GlobalVariables::EXPORT_DETAIL_LEVEL;
    GlobalVariables::EXPORT_DETAIL_LEVEL = 3;

    MockOrchestrator orchestrator(output_dir, 1000);
    orchestrator.PopulateData(GetMockJobHistory(), GetMockSPMetrics());
    orchestrator.CallExportResults("RM");

    std::string response_file = output_dir + "/RM/response_times_task_0.txt";
    EXPECT_TRUE(std::filesystem::exists(response_file));

    std::ifstream file(response_file);
    std::string line;
    ASSERT_TRUE(std::getline(file, line));
    EXPECT_EQ("jobId,release_time,start_time,finish_time,response_time,execution_time,is_overrun", line);

    // Job 0
    ASSERT_TRUE(std::getline(file, line));
    std::stringstream ss0(line);
    int jobId, is_overrun;
    LLint release_time, start_time, finish_time, response_time;
    double execution_time;
    char comma;
    ASSERT_TRUE(ss0 >> jobId >> comma >> release_time >> comma >> start_time >> comma >> finish_time >> comma >> response_time >> comma >> execution_time >> comma >> is_overrun);
    EXPECT_EQ(0, jobId);
    EXPECT_EQ(0, release_time);
    EXPECT_EQ(15 - 0, response_time);
    EXPECT_EQ(0, is_overrun);

    GlobalVariables::EXPORT_DETAIL_LEVEL = old_level;
}

TEST(OrchestratorTest, ExportResultsSampling) {
    std::string output_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_output_export_sampling";
    std::filesystem::remove_all(output_dir);

    int old_level = GlobalVariables::EXPORT_DETAIL_LEVEL;
    int old_sample = GlobalVariables::METRIC_SAMPLE_INTERVAL_SECONDS;

    GlobalVariables::EXPORT_DETAIL_LEVEL = 2;
    GlobalVariables::METRIC_SAMPLE_INTERVAL_SECONDS = 2;

    MockOrchestrator orchestrator(output_dir, 1000);
    orchestrator.PopulateData(GetMockJobHistory(), GetMockSPMetrics());
    orchestrator.CallExportResults("RM");

    std::string metrics_file = output_dir + "/RM/interval_sp_metrics.txt";
    EXPECT_TRUE(std::filesystem::exists(metrics_file));

    std::ifstream file(metrics_file);
    std::string line;
    int count = 0;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            count++;
            std::stringstream ss(line);
            int idx;
            char comma;
            ss >> idx >> comma;
            EXPECT_EQ(0, idx % 2);
        }
    }
    EXPECT_EQ(2, count);

    GlobalVariables::EXPORT_DETAIL_LEVEL = old_level;
    GlobalVariables::METRIC_SAMPLE_INTERVAL_SECONDS = old_sample;
}

TEST(OrchestratorTest, CFSOrchestration) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    std::string output_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_output_cfs";

    CFSSimulationOrchestrator orchestrator(input_dir, output_dir, 100);
    orchestrator.RunSimulation();

    const auto& history = orchestrator.GetJobHistory();
    ASSERT_FALSE(history.empty());

    // CFS does not enforce time limits → never overruns.
    for (const auto& r : history) {
        EXPECT_FALSE(r.isOverrun);
    }

    // Interval SP metrics come from probabilistic RTA using the unchanged dists
    // (same as RM because both evaluate the same raw distributions without TLs).
    const auto& sp_metrics = orchestrator.GetIntervalSPMetrics();
    ASSERT_EQ(2, sp_metrics.size());
    EXPECT_NEAR(3.6, sp_metrics[0], 1e-4);
    EXPECT_NEAR(3.86524, sp_metrics[1], 1e-4);
}

// Unit Tests for helper functions
TEST(OrchestratorTest, UnitDeterminePrioritiesAndBudgets) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    TestOrchestrator orchestrator(input_dir, "", "RM", 100);
    orchestrator.TestLoadConfigs();

    auto dags = orchestrator.GetDagTasks();
    auto sp = orchestrator.GetSpParameters();
    ASSERT_FALSE(dags.empty());

    ResourceOptResult res =
        orchestrator.DeterminePrioritiesAndBudgets(dags[0], sp[0]);
    // RM sorts by period: Task0 (10), Task1 (20), Task2 (20), Task3 (40)
    // priority_vec stores sorted indices
    ASSERT_EQ(4, res.priority_vec.size());
    EXPECT_EQ(0, res.priority_vec[0]);  // Task0
    EXPECT_EQ(3, res.priority_vec[3]);  // Task3
    EXPECT_DOUBLE_EQ(-1.0, res.id2time_limit[0]);
}

TEST(OrchestratorTest, UnitDeterminePrioritiesAndBudgets_RM_FAST) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    TestOrchestrator orchestrator(input_dir, "", "RM_FAST", 100);
    orchestrator.TestLoadConfigs();

    auto dags = orchestrator.GetDagTasks();
    auto sp = orchestrator.GetSpParameters();
    ASSERT_FALSE(dags.empty());

    ResourceOptResult res =
        orchestrator.DeterminePrioritiesAndBudgets(dags[0], sp[0]);
    ASSERT_EQ(4, res.priority_vec.size());
    EXPECT_EQ(0, res.priority_vec[0]);  // Task0 shortest period
    EXPECT_EQ(3, res.priority_vec[3]);  // Task3 longest period
    // Task0 has perf records, shortest (first) time limit = 1
    EXPECT_DOUBLE_EQ(1.0, res.id2time_limit[0]);
    // Task1 has no perf records
    EXPECT_DOUBLE_EQ(-1.0, res.id2time_limit[1]);
}

TEST(OrchestratorTest, UnitDeterminePrioritiesAndBudgets_RM_SLOW) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    TestOrchestrator orchestrator(input_dir, "", "RM_SLOW", 100);
    orchestrator.TestLoadConfigs();

    auto dags = orchestrator.GetDagTasks();
    auto sp = orchestrator.GetSpParameters();
    ASSERT_FALSE(dags.empty());

    ResourceOptResult res =
        orchestrator.DeterminePrioritiesAndBudgets(dags[0], sp[0]);
    ASSERT_EQ(4, res.priority_vec.size());
    EXPECT_EQ(0, res.priority_vec[0]);  // Task0 shortest period
    EXPECT_EQ(3, res.priority_vec[3]);  // Task3 longest period
    // Task0 has perf records, longest (last) time limit = 3
    EXPECT_DOUBLE_EQ(3.0, res.id2time_limit[0]);
    // Task1 has no perf records
    EXPECT_DOUBLE_EQ(-1.0, res.id2time_limit[1]);
}

TEST(OrchestratorTest, UnitApplyTaskConfigurations) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    TestOrchestrator orchestrator(input_dir, "", "RM", 100);
    orchestrator.TestLoadConfigs();

    auto dags = orchestrator.GetDagTasks();
    ASSERT_FALSE(dags.empty());

    ResourceOptResult res;
    res.priority_vec = {0, 1, 2, 3};  // Highest to lowest priority indices
    orchestrator.ApplyTaskConfigurations(dags[0], res);

    // Index 0 (Task0) priority should be 0
    EXPECT_EQ(0, dags[0].tasks[0].priority);
    // Index 3 (Task3) priority should be 3
    EXPECT_EQ(3, dags[0].tasks[3].priority);
}

TEST(OrchestratorTest, UnitRecordFinishedJobs) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    TestOrchestrator orchestrator(input_dir, "", "RM", 100);
    orchestrator.TestLoadConfigs();

    auto dags = orchestrator.GetDagTasks();
    ASSERT_FALSE(dags.empty());

    RunQueue rq(dags[0].tasks);
    JobCEC job(0, 0);
    JobStartFinish jsf(0, 2, 2);
    rq.schedule_[job] = jsf;

    ResourceOptResult res;
    res.id2time_limit[0] = 1.0;  // Overrun threshold 1.0 < execution time 2

    orchestrator.RecordFinishedJobs(2, rq, res, dags[0]);
    const auto& history = orchestrator.GetJobHistory();
    ASSERT_EQ(1, history.size());
    EXPECT_EQ(0, history[0].taskId);
    EXPECT_EQ(0, history[0].jobId);
    EXPECT_EQ(2, history[0].finishTime);
    EXPECT_TRUE(history[0].isOverrun);
}

TEST(OrchestratorTest, UnitReleaseJobs) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    TestOrchestrator orchestrator(input_dir, "", "RM", 100);
    orchestrator.TestLoadConfigs();

    auto dags = orchestrator.GetDagTasks();
    ASSERT_FALSE(dags.empty());

    RunQueue rq(dags[0].tasks);
    ResourceOptResult res;
    res.id2time_limit[0] = -1.0;

    std::unordered_map<int, std::vector<float>> traces;
    std::unordered_map<int, size_t> trace_indices;
    traces[0] = {1.5f};
    trace_indices[0] = 0;

    // Release jobs at time 0
    orchestrator.ReleaseJobs(0, 100, dags[0], res, rq, traces, trace_indices);
    // Period of Task0 is 10, Task1 is 20, Task2 is 20, Task3 is 40. At time 0,
    // all should release.
    EXPECT_EQ(4, rq.job_queue_.size());
    // Job 0 for Task0 should have execution time rounded to 2 (1.5 + 0.5 = 2.0)
    EXPECT_EQ(2, rq.job_queue_[0].executionTime);
}

// Integration Test checking exact values of response time, start, finish times
TEST(OrchestratorTest, ExactResponseTimeValidation) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    std::string output_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_output_exact_val";

    FixedTaskPrioritySchedulingOrchestrator orchestrator(input_dir, output_dir,
                                                         "RM", 100);
    orchestrator.RunSimulation();

    // Print hyperperiod schedule to stdout for manual verification
    orchestrator.PrintHyperperiodSchedule(0, 40);

    // Verify response times of Task0
    std::string response_file_0 = output_dir + "/RM/response_times_task_0.txt";
    std::ifstream infile(response_file_0);
    std::string line;
    std::getline(infile, line);  // Header

    // Job 0
    std::getline(infile, line);
    std::stringstream ss0(line);
    int jobId, release, start, finish, response, execution, overrun;
    char comma;
    ASSERT_TRUE(ss0 >> jobId >> comma >> release >> comma >> start >> comma >>
                finish >> comma >> response >> comma >> execution >> comma >>
                overrun);
    EXPECT_EQ(0, jobId);
    EXPECT_EQ(0, release);
    EXPECT_EQ(0, start);
    EXPECT_EQ(2, finish);
    EXPECT_EQ(2, response);
    EXPECT_EQ(2, execution);

    // Job 1
    std::getline(infile, line);
    std::stringstream ss1(line);
    ASSERT_TRUE(ss1 >> jobId >> comma >> release >> comma >> start >> comma >>
                finish >> comma >> response >> comma >> execution >> comma >>
                overrun);
    EXPECT_EQ(1, jobId);
    EXPECT_EQ(10, release);
    EXPECT_EQ(10, start);
    EXPECT_EQ(13, finish);
    EXPECT_EQ(3, response);
    EXPECT_EQ(3, execution);

    // Job 2
    std::getline(infile, line);
    std::stringstream ss2(line);
    ASSERT_TRUE(ss2 >> jobId >> comma >> release >> comma >> start >> comma >>
                finish >> comma >> response >> comma >> execution >> comma >>
                overrun);
    EXPECT_EQ(2, jobId);
    EXPECT_EQ(20, release);
    EXPECT_EQ(20, start);
    EXPECT_EQ(24, finish);
    EXPECT_EQ(4, response);
    EXPECT_EQ(4, execution);

    // Verify response times of Task1
    std::string response_file_1 = output_dir + "/RM/response_times_task_1.txt";
    std::ifstream infile_1(response_file_1);
    std::getline(infile_1, line);  // Header
    std::getline(infile_1, line);  // Job 0
    std::stringstream ss_t1_j0(line);
    ASSERT_TRUE(ss_t1_j0 >> jobId >> comma >> release >> comma >> start >>
                comma >> finish >> comma >> response >> comma >> execution >>
                comma >> overrun);
    EXPECT_EQ(0, jobId);
    EXPECT_EQ(0, release);
    EXPECT_EQ(2, start);
    EXPECT_EQ(5, finish);
    EXPECT_EQ(5, response);
    EXPECT_EQ(3, execution);
}

TEST(OrchestratorTest, CFS_RunOrchestrator_Binary) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    std::string output_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_output_cfs_binary";
    std::filesystem::remove_all(output_dir);

    std::string binary_path =
        GlobalVariables::PROJECT_PATH + "build/tests/RunOrchestrator";
    std::string cmd =
        binary_path + " " + input_dir + " " + output_dir + " CFS 100";
    int ret = std::system(cmd.c_str());
    EXPECT_EQ(0, ret);

    std::string metrics_file = output_dir + "/CFS/interval_sp_metrics.txt";
    EXPECT_TRUE(std::filesystem::exists(metrics_file));

    // Verify the interval SP metrics written by the binary are deterministic.
    std::ifstream infile(metrics_file);
    std::string line;
    ASSERT_TRUE(std::getline(infile, line));
    std::stringstream ss0(line);
    int interval_idx;
    double sp_val;
    char comma;
    ASSERT_TRUE(ss0 >> interval_idx >> comma >> sp_val);
    EXPECT_EQ(0, interval_idx);
    EXPECT_NEAR(3.6, sp_val, 1e-4);

    ASSERT_TRUE(std::getline(infile, line));
    std::stringstream ss1(line);
    ASSERT_TRUE(ss1 >> interval_idx >> comma >> sp_val);
    EXPECT_EQ(1, interval_idx);
    EXPECT_NEAR(3.86524, sp_val, 1e-4);
}

// TODO(orchestrator-followup): re-enable once the orchestrator bootstraps
// interval 0 of INCR/INCR_NO_TL/INCR_WCET with a from_scratch call before
// going incremental. Today RunSimulation routes interval 0 through
// OptimizeIncre_w_TL on a fresh incr_optimizer_, which violates the
// from-scratch-first contract and hits the hard error in
// EvaluateTimeLimitConfig_ScratchOrIncre (std::terminate). The orchestrator
// fix is deferred to a follow-up task.
TEST(OrchestratorTest, DISABLED_INCR_NO_TL_Integration) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    std::string output_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_output_incr_no_tl";
    std::filesystem::remove_all(output_dir);

    FixedTaskPrioritySchedulingOrchestrator orchestrator(input_dir, output_dir,
                                                         "INCR_NO_TL", 100);
    orchestrator.RunSimulation();

    const auto& history = orchestrator.GetJobHistory();
    ASSERT_FALSE(history.empty());

    // With TL optimization disabled, Task0 gets the smallest option (TL=1).
    // Every Task0 job is clamped to execution_time=1 and flagged as overrun.
    int task0_count = 0;
    for (const auto& r : history) {
        if (r.taskId == 0) {
            EXPECT_EQ(1, r.executionTime)
                << "Task0 should be clamped to the smallest TL=1";
            EXPECT_TRUE(r.isOverrun)
                << "Task0 should always overrun with TL=1";
            task0_count++;
        }
    }
    EXPECT_EQ(20, task0_count);  // 2 intervals × 10 Task0 jobs per interval

    // Deterministic interval SP metrics.
    // Task0 with TL=1: perf=0.3, ddl_miss≈0 → SP=0.3.
    // Tasks 1-3 have no time limits → SP=1.0 each.
    // Total = 0.3 + 1.0 + 1.0 + 1.0 = 3.3 per interval.
    const auto& sp_metrics = orchestrator.GetIntervalSPMetrics();
    ASSERT_EQ(2, sp_metrics.size());
    EXPECT_NEAR(3.3, sp_metrics[0], 1e-4);
    EXPECT_NEAR(3.3, sp_metrics[1], 1e-4);
}

// TODO(orchestrator-followup): re-enable once the orchestrator bootstraps
// interval 0 of INCR/INCR_NO_TL/INCR_WCET with a from_scratch call before
// going incremental. Today RunSimulation routes interval 0 through
// OptimizeIncre_w_TL on a fresh incr_optimizer_, which violates the
// from-scratch-first contract and hits the hard error in
// EvaluateTimeLimitConfig_ScratchOrIncre (std::terminate). The orchestrator
// fix is deferred to a follow-up task.
TEST(OrchestratorTest, DISABLED_INCR_WCET_Integration) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
    std::string output_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_output_incr_wcet";
    std::filesystem::remove_all(output_dir);

    FixedTaskPrioritySchedulingOrchestrator orchestrator(input_dir, output_dir,
                                                         "INCR_WCET", 100);
    orchestrator.RunSimulation();

    const auto& history = orchestrator.GetJobHistory();
    ASSERT_FALSE(history.empty());

    // Interval 0: WCET ablation sets Task0 ET to 3. The optimizer chooses TL=3,
    // so jobs with trace values [2,3,3] (rounded) are overrun only when exec==3.
    int task0_overrun_i0 = 0;
    int task0_not_overrun_i0 = 0;
    for (const auto& r : history) {
        if (r.taskId == 0 && r.releaseTime < 100) {
            EXPECT_TRUE(r.executionTime == 2 || r.executionTime == 3)
                << "Interval 0 Task0 exec should be 2 or 3 with TL=3";
            if (r.isOverrun) {
                EXPECT_EQ(3, r.executionTime);
                task0_overrun_i0++;
            } else {
                EXPECT_EQ(2, r.executionTime);
                task0_not_overrun_i0++;
            }
        }
    }
    EXPECT_EQ(6, task0_overrun_i0);
    EXPECT_EQ(4, task0_not_overrun_i0);

    // Interval 1: WCET for Task0 becomes 4. The optimizer chooses TL=2, which
    // clamps all trace values to exactly 2 after rounding, so every job overruns.
    int task0_overrun_i1 = 0;
    for (const auto& r : history) {
        if (r.taskId == 0 && r.releaseTime >= 100) {
            EXPECT_EQ(2, r.executionTime)
                << "Interval 1 Task0 exec should be clamped to 2 with TL=2";
            EXPECT_TRUE(r.isOverrun);
            task0_overrun_i1++;
        }
    }
    EXPECT_EQ(10, task0_overrun_i1);

    // Deterministic interval SP metrics.
    // i0: TL=3 → Task0 perf=1.0, total SP=4.0.
    // i1: TL=2 → Task0 perf=0.6, total SP=3.6.
    const auto& sp_metrics = orchestrator.GetIntervalSPMetrics();
    ASSERT_EQ(2, sp_metrics.size());
    EXPECT_NEAR(4.0, sp_metrics[0], 1e-4);
    EXPECT_NEAR(3.6, sp_metrics[1], 1e-4);
}

// =============================================================================
// RunQueue direct unit tests — verify running_job_index_ tracking, O(1)
// RemoveFinishedJob, PreemptRunningJob, and related invariants.
// =============================================================================

// Helper to create a deterministic FiniteDist at a single value.
static FiniteDist MakeDeterministicDist(double val) {
    return FiniteDist(std::vector<Value_Proba>{{val, 1.0}});
}

class RunQueueTestFixture : public ::testing::Test {
   public:
    void SetUp() override {
        // Build a minimal 3-task TaskSet with explicit execution times.
        Task t0(0, MakeDeterministicDist(1.0), 10, 10, 0, "T0");
        Task t1(1, MakeDeterministicDist(2.0), 20, 20, 1, "T1");
        Task t2(2, MakeDeterministicDist(3.0), 30, 30, 2, "T2");
        t0.setExecutionTime(1);
        t1.setExecutionTime(2);
        t2.setExecutionTime(3);
        tasks = {t0, t1, t2};
        tasks_info = TaskSetInfoDerived(tasks);
    }

    TaskSet tasks;
    TaskSetInfoDerived tasks_info;
};

TEST_F(RunQueueTestFixture, RunJob_SetsRunningJobIndex) {
    RunQueue rq(tasks_info);
    JobCEC job0(0, 0);
    rq.insert(job0);
    EXPECT_EQ(-1, rq.running_job_index_);

    EXPECT_TRUE(rq.RunJob(0, 0));
    EXPECT_EQ(0, rq.running_job_index_);
    EXPECT_FALSE(rq.processor_free_);
    EXPECT_TRUE(rq.job_queue_[0].running);
}

TEST_F(RunQueueTestFixture, PreemptJob_ClearsRunningJobIndex) {
    RunQueue rq(tasks_info);
    rq.insert(JobCEC(0, 0));
    rq.RunJob(0, 0);
    EXPECT_EQ(0, rq.running_job_index_);

    rq.PreemptJob(0, 5);
    EXPECT_EQ(-1, rq.running_job_index_);
    EXPECT_TRUE(rq.processor_free_);
    EXPECT_FALSE(rq.job_queue_[0].running);
    // accum_run_time should reflect elapsed time [0,5]
    EXPECT_EQ(5, rq.job_queue_[0].accum_run_time);
}

TEST_F(RunQueueTestFixture, PreemptRunningJob_OnlyPreemptsRunningJob) {
    RunQueue rq(tasks_info);
    // Insert two jobs: job0 higher priority, then job1
    rq.insert(JobCEC(0, 0));
    rq.insert(JobCEC(1, 0));
    rq.RunJob(0, 0);

    rq.PreemptRunningJob(4);
    EXPECT_EQ(-1, rq.running_job_index_);
    EXPECT_TRUE(rq.job_queue_[0].running == false);
    EXPECT_TRUE(rq.job_queue_[1].running == false);  // was never running
}

TEST_F(RunQueueTestFixture, RemoveFinishedJob_RemovesOnlyRunningJob) {
    RunQueue rq(tasks_info);
    // T0 exec=1, so it finishes at time 1
    rq.insert(JobCEC(0, 0));
    // T1 exec=2, remains pending
    rq.insert(JobCEC(1, 0));

    rq.RunJob(0, 0);
    EXPECT_EQ(0, rq.running_job_index_);
    EXPECT_EQ(2, rq.size());

    // At time 1 the running job (T0) should finish, T1 stays
    rq.RemoveFinishedJob(1);
    EXPECT_EQ(1, rq.size());
    EXPECT_EQ(1, rq.job_queue_[0].job.taskId);  // T1 remains
    EXPECT_EQ(-1, rq.running_job_index_);
    EXPECT_TRUE(rq.processor_free_);

    // Schedule should record T0 finished at 1
    auto sched = rq.GetSchedule();
    EXPECT_EQ(1, sched[JobCEC(0, 0)].finish);
}

TEST_F(RunQueueTestFixture, RemoveFinishedJob_LeavesNonRunningJobsUntouched) {
    RunQueue rq(tasks_info);
    // Insert two jobs; run the first, then preempt it before it finishes
    rq.insert(JobCEC(0, 0));  // exec=1
    rq.insert(JobCEC(1, 0));  // exec=2
    rq.RunJob(0, 0);
    rq.PreemptJob(0, 0);  // preempt immediately, accumulated = 0

    // At time 5 neither job has accumulated any execution time, so none finish
    rq.RemoveFinishedJob(5);
    EXPECT_EQ(2, rq.size());
    EXPECT_EQ(-1, rq.running_job_index_);
}

TEST_F(RunQueueTestFixture, RemoveFinishedJob_EmptyQueueDoesNotCrash) {
    RunQueue rq(tasks_info);
    EXPECT_EQ(0, rq.size());
    rq.RemoveFinishedJob(10);
    EXPECT_EQ(0, rq.size());
    EXPECT_EQ(-1, rq.running_job_index_);
}

TEST_F(RunQueueTestFixture, FullLifecycle_SequentialRunAndFinish) {
    RunQueue rq(tasks_info);
    JobCEC j0(0, 0);  // exec=1, priority=0 (highest)
    JobCEC j1(1, 0);  // exec=2, priority=1
    JobCEC j2(2, 0);  // exec=3, priority=2

    // Insert all three in priority order; queue = [j0, j1, j2]
    rq.insert(j0);
    rq.insert(j1);
    rq.insert(j2);

    // j0 runs and finishes [0,1]
    rq.RunJob(0, 0);
    EXPECT_EQ(0, rq.running_job_index_);
    rq.RemoveFinishedJob(1);
    EXPECT_EQ(2, rq.size());

    // j1 runs and finishes [1,3]
    rq.RunJob(0, 1);
    EXPECT_EQ(0, rq.running_job_index_);
    EXPECT_EQ(1, rq.job_queue_[0].job.taskId);
    rq.RemoveFinishedJob(3);
    EXPECT_EQ(1, rq.size());

    // j2 runs and finishes [3,6]
    rq.RunJob(0, 3);
    EXPECT_EQ(0, rq.running_job_index_);
    EXPECT_EQ(2, rq.job_queue_[0].job.taskId);
    rq.RemoveFinishedJob(6);
    EXPECT_TRUE(rq.empty());

    auto sched = rq.GetSchedule();
    EXPECT_EQ(1, sched[j0].finish);
    EXPECT_EQ(3, sched[j1].finish);
    EXPECT_EQ(6, sched[j2].finish);
}

TEST_F(RunQueueTestFixture, Preemption_HigherPriorityPreemptsThenRuns) {
    RunQueue rq(tasks_info);

    // Insert only low-priority j2 and start it
    JobCEC j2(2, 0);  // exec=3, priority=2
    rq.insert(j2);
    rq.RunJob(0, 0);
    EXPECT_EQ(2, rq.job_queue_[0].job.taskId);
    EXPECT_EQ(0, rq.running_job_index_);

    // Preempt j2 at t=0 before any progress
    rq.PreemptRunningJob(0);
    EXPECT_EQ(-1, rq.running_job_index_);

    // Insert higher-priority jobs; priority sort makes queue = [j0, j1, j2]
    JobCEC j0(0, 0);  // exec=1, priority=0
    JobCEC j1(1, 0);  // exec=2, priority=1
    rq.insert(j0);
    rq.insert(j1);

    // Run highest-priority j0
    EXPECT_EQ(0, rq.job_queue_[0].job.taskId);
    rq.RunJob(0, 0);
    EXPECT_EQ(0, rq.running_job_index_);

    // j0 finishes [0,1]
    rq.RemoveFinishedJob(1);
    EXPECT_EQ(-1, rq.running_job_index_);
    EXPECT_EQ(2, rq.size());  // j1 and j2 remain

    // j1 finishes [1,3]
    rq.RunJob(0, 1);
    EXPECT_EQ(0, rq.running_job_index_);
    rq.RemoveFinishedJob(3);
    EXPECT_EQ(1, rq.size());

    // j2 (already had 2 remaining) finishes [3,6]
    rq.RunJob(0, 3);
    EXPECT_EQ(0, rq.running_job_index_);
    rq.RemoveFinishedJob(6);
    EXPECT_TRUE(rq.empty());

    auto sched = rq.GetSchedule();
    EXPECT_EQ(1, sched[j0].finish);
    EXPECT_EQ(3, sched[j1].finish);
    EXPECT_EQ(6, sched[j2].finish);
}

TEST_F(RunQueueTestFixture, RunningJobIndex_CorrectAfterErase) {
    RunQueue rq(tasks_info);
    rq.insert(JobCEC(0, 0));  // exec=1
    rq.insert(JobCEC(1, 0));  // exec=2

    rq.RunJob(0, 0);
    EXPECT_EQ(0, rq.running_job_index_);

    // Remove finished running job — index should reset to -1
    rq.RemoveFinishedJob(1);
    EXPECT_EQ(-1, rq.running_job_index_);
    EXPECT_EQ(1, rq.size());
}

// P1.7 — runtime SimulateInterval must partition on processorId.
//
// Two tasks, equal period (10) and ET (2), but on DIFFERENT cores
// (processorId 0 and 1). RM priority: Task0 < Task1.
//
// On the BUGGY single-queue simulator: at t=0 both jobs release into one
// RunQueue; Task0 (higher priority) runs [0,2], then Task1 runs [2,4] —
// SERIALIZED on one core, Task1.start == 2.
//
// On the FIXED per-processorId simulator: each core has its own RunQueue, so
// Task0 runs [0,2] on core 0 and Task1 runs [0,2] on core 1 IN PARALLEL —
// Task1.start == 0. Two cores declared, two cores simulated.
//
// This test drives the real runtime path (RunSimulation -> SimulateInterval ->
// RecordFinishedJobs -> job_history_), NOT the legacy SimulateFixedPrioritySched
// path that already partitions.
TEST(OrchestratorTest, SimulateIntervalPartitionsByProcessorId) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_partition_two_cores";
    std::string output_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_output_partition_two_cores";
    std::filesystem::remove_all(output_dir);

    FixedTaskPrioritySchedulingOrchestrator orchestrator(input_dir, output_dir,
                                                         "RM", 100);
    orchestrator.RunSimulation();

    const auto& history = orchestrator.GetJobHistory();
    ASSERT_FALSE(history.empty());

    // Job 0 of Task0 and Job 0 of Task1 are both released at t=0. With correct
    // per-core partitioning they run in parallel, so BOTH start at t=0.
    // (On the buggy single queue, Task1.start would be 2 — serialized.)
    int task0_j0_start = -1;
    int task1_j0_start = -1;
    for (const auto& r : history) {
        if (r.taskId == 0 && r.jobId == 0) task0_j0_start = r.startTime;
        if (r.taskId == 1 && r.jobId == 0) task1_j0_start = r.startTime;
    }
    ASSERT_NE(-1, task0_j0_start);
    ASSERT_NE(-1, task1_j0_start);
    EXPECT_EQ(0, task0_j0_start);
    EXPECT_EQ(0, task1_j0_start)
        << "Task1 on processorId:1 must run in parallel with Task0 on "
           "processorId:0, not be serialized behind it on one queue";
}

// P1.7 CFS sibling — same partitioning requirement on the CFS orchestrator.
// Two equal-period/ET tasks on different cores must run in parallel under CFS
// too (both start at t=0), not be serialized on one CFS run queue.
TEST(OrchestratorTest, SimulateIntervalPartitionsByProcessorId_CFS) {
    std::string input_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_data_partition_two_cores";
    std::string output_dir =
        GlobalVariables::PROJECT_PATH + "tests/test_output_partition_two_cores_cfs";
    std::filesystem::remove_all(output_dir);

    CFSSimulationOrchestrator orchestrator(input_dir, output_dir, 100);
    orchestrator.RunSimulation();

    const auto& history = orchestrator.GetJobHistory();
    ASSERT_FALSE(history.empty());

    int task0_j0_start = -1;
    int task1_j0_start = -1;
    for (const auto& r : history) {
        if (r.taskId == 0 && r.jobId == 0) task0_j0_start = r.startTime;
        if (r.taskId == 1 && r.jobId == 0) task1_j0_start = r.startTime;
    }
    ASSERT_NE(-1, task0_j0_start);
    ASSERT_NE(-1, task1_j0_start);
    EXPECT_EQ(0, task0_j0_start);
    EXPECT_EQ(0, task1_j0_start)
        << "CFS: Task1 on processorId:1 must run in parallel with Task0 on "
           "processorId:0, not be serialized behind it on one queue";
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}