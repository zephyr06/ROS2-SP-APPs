// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/TaskModel/execution_time_estimator.h"
#include "sources/Utils/Parameters.h"
using ::testing::AtLeast;  // #1
using ::testing::Return;
using namespace std;
// using namespace SP_OPT_PA;
// using namespace GlobalVariables;

TEST(extractIndex, V1) {
    std::string line =
        "Execution time slam message:slam 64::0.91871564800000005135";
    EXPECT_EQ(64, extractIndex(line));
    EXPECT_EQ(0.91871564800000005135, extractNumber(line));
}
TEST(BasicExample, v1) {
    std::string slam_ext_file =
        GlobalVariables::PROJECT_PATH +
        "/TaskData/AnalyzeSP_Metric/slam_execution_time.txt";

    std::vector<double> execution_times_exp = {
        1.2945933119999999406, 1.2930264000000000202, 16.393618111999998632,
        0.53181478400000004036, 0.91871564800000005135};
    int miss_count_exp = 2.0;
    std::vector<double> exe_data_actual;
    double miss_count_actual;
    std::tie(exe_data_actual, miss_count_actual) =
        ReadExtTimeData(slam_ext_file, 5);
    EXPECT_EQ(exe_data_actual, execution_times_exp);
    EXPECT_EQ(miss_count_actual, miss_count_exp);
}

TEST(BasicExample, v2) {
    std::string slam_ext_file =
        GlobalVariables::PROJECT_PATH +
        "/TaskData/AnalyzeSP_Metric/slam_execution_time2.txt";

    std::vector<double> execution_times_exp = {
        16.393618111999998632, 0.53181478400000004036, 0.91871564800000005135};
    int miss_count_exp = 2.0;
    std::vector<double> exe_data_actual;
    double miss_count_actual;
    std::tie(exe_data_actual, miss_count_actual) =
        ReadExtTimeData(slam_ext_file, 5);
    EXPECT_EQ(exe_data_actual, execution_times_exp);
    EXPECT_EQ(miss_count_actual, miss_count_exp);
}

int main(int argc, char **argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}