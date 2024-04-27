#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>
#include <iomanip>
#include <iostream>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Utils/profilier.h"

class AppBase {
   public:
    AppBase(std::string app_name) : app_name_(app_name) {}
    // function arguments not used for now
    virtual void run(int) { ; }

    // data member
    std::string app_name_;
};

template <typename AppBase>
class PeriodicReleaser {
   public:
    PeriodicReleaser(int period_ms, int release_total, const AppBase& app)
        : app_(app),
          period_ms_(period_ms),
          release_total_left_(release_total) {}

    void caller(const boost::system::error_code&,
                boost::asio::deadline_timer& t) {
        if (release_total_left_ == 0)
            return;
        app_.run(0);
        release_total_left_--;
        t.expires_at(t.expires_at() +
                     boost::posix_time::milliseconds(period_ms_));
        // if (++count < 100)
        t.async_wait(boost::bind(&PeriodicReleaser<AppBase>::caller, this,
                                 boost::asio::placeholders::error,
                                 boost::ref(t)));
    }

    void release() {
        boost::asio::io_service io;
        boost::asio::deadline_timer t(
            io, boost::posix_time::milliseconds(period_ms_));
        t.async_wait(boost::bind(&PeriodicReleaser<AppBase>::caller, this,
                                 boost::asio::placeholders::error,
                                 boost::ref(t)));
        io.run();
    }

    AppBase app_;
    int period_ms_;
    int release_total_left_;
};

void busySpinForSeconds(int ms) {
    auto startTime = std::chrono::high_resolution_clock::now();
    auto endTime = startTime + std::chrono::milliseconds(ms);

    while (std::chrono::high_resolution_clock::now() < endTime) {
        // Busy spin
    }
}

class AppTest : public AppBase {
   public:
    AppTest() : AppBase("AppTest") {}
    void run(int) override { std::cout << "Run one time!\n"; }
};

TEST(PeriodicReleaser, v1) {
    AppTest app;
    PeriodicReleaser<AppTest> releaser(100, 5, app);
    auto start = std::chrono::high_resolution_clock::now();
    releaser.release();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    EXPECT_NEAR(100 * (5 + 1), duration.count(), 5e-1);
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}