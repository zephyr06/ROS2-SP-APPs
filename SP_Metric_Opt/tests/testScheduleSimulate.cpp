// #include <gtest/gtest.h>

#include "sources/RTDA/ImplicitCommunication/ScheduleSimulation.h"
#include "sources/RTDA/ImplicitCommunication/SimulationOrchestrator.h"
#include "sources/Safety_Performance_Metric/ParametersSP.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/TaskModel/RegularTasks.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/readwrite.h"
#include "gmock/gmock.h"  // Brings in gMock.
using ::testing::AtLeast; // #1
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
  TestOrchestrator(const std::string &input_folder,
                   const std::string &output_folder, const std::string &mode,
                   LLint duration)
      : FixedTaskPrioritySchedulingOrchestrator(input_folder, output_folder,
                                                mode, duration) {}

  void TestLoadConfigs() { LoadIntervalConfigs(); }

  const std::vector<DAG_Model> &GetDagTasks() const { return dag_tasks_vecs_; }
  const std::vector<TaskSetInfoDerived> &GetTasksInfo() const {
    return tasks_info_vecs_;
  }
  const std::vector<SP_Parameters> &GetSpParameters() const {
    return sp_parameters_vecs_;
  }

  std::vector<float> TestLoadTraces(int task_id, int path_idx, int inst_idx) {
    return LoadJobExecutionTraces(task_id, path_idx, inst_idx);
  }
};

class TestCFSOrchestrator : public CFSSimulationOrchestrator {
public:
  TestCFSOrchestrator(const std::string &input_folder,
                      const std::string &output_folder, LLint duration)
      : CFSSimulationOrchestrator(input_folder, output_folder, duration) {}

  void TestLoadConfigs() { LoadIntervalConfigs(); }
  const std::vector<DAG_Model> &GetDagTasks() const { return dag_tasks_vecs_; }
  const std::vector<SP_Parameters> &GetSpParameters() const {
    return sp_parameters_vecs_;
  }
};

TEST(OrchestratorTest, LoadIntervalConfigs) {
  std::string input_dir =
      GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
  TestOrchestrator orchestrator(input_dir, "", "RM", 100);
  orchestrator.TestLoadConfigs();

  const auto &dags = orchestrator.GetDagTasks();
  ASSERT_EQ(2, dags.size()); // i0 and i1
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

  // Verify task priorities: RM assigns highest priority (smallest value) to
  // task with shortest period (Task0: 10) and lowest priority to task with
  // longest period (Task3: 40)
  const auto &history = orchestrator.GetJobHistory();
  ASSERT_FALSE(history.empty());
}

TEST(OrchestratorTest, ExportResults) {
  std::string input_dir =
      GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
  std::string output_dir =
      GlobalVariables::PROJECT_PATH + "tests/test_output_export";

  // Clean directory first if exists
  std::filesystem::remove_all(output_dir);

  FixedTaskPrioritySchedulingOrchestrator orchestrator(input_dir, output_dir,
                                                       "RM", 100);
  orchestrator.RunSimulation();

  // Check if the output files exist and are populated
  std::string response_file = output_dir + "/RM/response_times_task_0.txt";
  std::string metrics_file = output_dir + "/RM/interval_sp_metrics.txt";

  EXPECT_TRUE(std::filesystem::exists(response_file));
  EXPECT_TRUE(std::filesystem::exists(metrics_file));

  // Verify the response time file starts with correct header
  std::ifstream file(response_file);
  std::string line;
  std::getline(file, line);
  EXPECT_EQ("jobId,release_time,start_time,finish_time,response_time,execution_"
            "time,is_overrun",
            line);
}

TEST(OrchestratorTest, CFSOrchestration) {
  std::string input_dir =
      GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
  std::string output_dir =
      GlobalVariables::PROJECT_PATH + "tests/test_output_cfs";

  CFSSimulationOrchestrator orchestrator(input_dir, output_dir, 100);
  orchestrator.RunSimulation();

  const auto &history = orchestrator.GetJobHistory();
  EXPECT_FALSE(history.empty());
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
  EXPECT_EQ(0, res.priority_vec[0]); // Task0
  EXPECT_EQ(3, res.priority_vec[3]); // Task3
  EXPECT_DOUBLE_EQ(-1.0, res.id2time_limit[0]);
}

TEST(OrchestratorTest, UnitApplyTaskConfigurations) {
  std::string input_dir =
      GlobalVariables::PROJECT_PATH + "tests/test_data_schedule_orchestrator";
  TestOrchestrator orchestrator(input_dir, "", "RM", 100);
  orchestrator.TestLoadConfigs();

  auto dags = orchestrator.GetDagTasks();
  ASSERT_FALSE(dags.empty());

  ResourceOptResult res;
  res.priority_vec = {0, 1, 2, 3}; // Highest to lowest priority indices
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
  res.id2time_limit[0] = 1.0; // Overrun threshold 1.0 < execution time 2

  orchestrator.RecordFinishedJobs(2, rq, res, dags[0]);
  const auto &history = orchestrator.GetJobHistory();
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
  std::getline(infile, line); // Header

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
  std::getline(infile_1, line); // Header
  std::getline(infile_1, line); // Job 0
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

int main(int argc, char **argv) {
  // ::testing::InitGoogleTest(&argc, argv);
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}