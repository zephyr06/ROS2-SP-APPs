// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Utils/Parameters.h"
#include "sources/UtilsForROS2/execution_time_profiler.h"
using ::testing::AtLeast;  // #1
using ::testing::Return;
using namespace std;
using namespace GlobalVariables;
TEST(BasicExample, v1) {
    ExecutionTimeProfiler ext_profiler;
    for (int k = 0; k < 5; k++) {
        ext_profiler.start();
        int a = 1;
        for (int i = 0; i < 1e8; i++) {
            a++;
            a++;
            int b = a * 2;
            a = b * 2;
        }
        ext_profiler.end();
        cout << "Execution time: " << ext_profiler.get_exe_time()
             << " seconds\n";
    }
}

int main(int argc, char **argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}