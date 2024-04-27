#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>
#include <iomanip>
#include <iostream>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/profilier.h"
#include "sources/UtilsForROS2/Publisher.h"
#include "sources/UtilsForROS2/execution_time_profiler.h"
#include "sources/UtilsForROS2/profile_and_record_time.h"
using namespace std;
using namespace SP_OPT_PA;
TEST(PeriodicReleaser, v1) {
    AppTest app;
    PeriodicReleaser<AppTest> releaser(100, 5, app);
    auto start = std::chrono::high_resolution_clock::now();
    releaser.release();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    EXPECT_NEAR(100 * (5 + 1), duration.count(), 1e0);
}
TEST(read_tassks, time_limit) {
    string file_path =
        GlobalVariables::PROJECT_PATH + "TaskData/test_robotics_v16.yaml";
    DAG_Model dag_tasks = ReadDAG_Tasks(file_path);
    EXPECT_EQ(3000, dag_tasks.tasks[0].total_running_time);
}
int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}