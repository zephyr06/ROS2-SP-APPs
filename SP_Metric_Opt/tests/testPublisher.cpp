#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>
#include <iomanip>
#include <iostream>
namespace bpt = boost::posix_time;
namespace asio = boost::asio;
void busySpinForSeconds(int ms) {
    auto startTime = std::chrono::high_resolution_clock::now();
    auto endTime = startTime + std::chrono::milliseconds(ms);

    while (std::chrono::high_resolution_clock::now() < endTime) {
        // Busy spin
    }
}
int i = 0;
void f() {
    std::cout << "Called at " << bpt::microsec_clock::local_time().time_of_day()
              << '\n';
    // sleep(1.5);
    if (i % 2 == 0)
        busySpinForSeconds(1500);
    else
        busySpinForSeconds(500);
    i++;
}

void caller(const boost::system::error_code&, asio::deadline_timer& t,
            int& count) {
    f();
    t.expires_at(t.expires_at() + bpt::seconds(1));
    if (++count < 100)
        t.async_wait(boost::bind(caller, asio::placeholders::error,
                                 boost::ref(t), boost::ref(count)));
}

int main() {
    asio::io_service io;
    asio::deadline_timer t(io, bpt::seconds(1));
    int count = 0;
    t.async_wait(boost::bind(caller, asio::placeholders::error, boost::ref(t),
                             boost::ref(count)));
    io.run();
}