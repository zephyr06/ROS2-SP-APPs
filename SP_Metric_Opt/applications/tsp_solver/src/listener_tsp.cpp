#include "TSPSolver.h"
#include "sources/UtilsForROS2/Publisher.h"

class TSPApp : public AppBase {
   public:
    TSPApp() : AppBase("tsp") {}
    void run(int msg_cnt) override { callTSP(); }
};

// TODO: read period
int main(int argc, char* argv[]) {
    AppTest app;
    PeriodicReleaser<AppTest> releaser(1000, 5, app);
    releaser.release();
    return 0;
}