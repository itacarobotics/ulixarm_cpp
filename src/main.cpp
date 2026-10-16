#include "ulixarm_cpp/ulixarm.hpp"

#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <limits>

int main()
{
    UlixArm robot(/*gripper*/ true);

    try
    {
        robot.connect("/dev/ttyACM0");
        robot.enable();
        robot.refresh();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Robot initialization failed: " << e.what() << '\n';
        return -1;
    }

    Joints q   = Joints::Zero();
    Joints dq  = Joints::Zero();
    Joints tau = Joints::Zero();
    Joints kp  = Joints::Zero();
    Joints kd  = Joints::Zero();

    using Clock = std::chrono::steady_clock;

    double min_latency = std::numeric_limits<double>::max();
    double max_latency = 0.0;
    double sum_latency = 0.0;
    double sum_squared = 0.0;

    long long count = 0;

    constexpr long long PRINT_INTERVAL = 1000;

    std::cout << std::fixed << std::setprecision(2);

    while (true)
    {
        const auto start = Clock::now();

        robot.control(q, dq, tau, kp, kd, true);

        const auto end = Clock::now();

        const double latency_us =
            std::chrono::duration<double, std::micro>(
                end - start
            ).count();

        min_latency = std::min(min_latency, latency_us);
        max_latency = std::max(max_latency, latency_us);
        sum_latency += latency_us;
        sum_squared += latency_us * latency_us;

        ++count;

        if (count % PRINT_INTERVAL == 0)
        {
            const double avg_latency = sum_latency / count;

            const double variance =
                std::max(0.0, sum_squared / count
                              - avg_latency * avg_latency);

            const double stddev_latency = std::sqrt(variance);

            std::cout
                << "Iterations: " << count
                << " | Current: " << latency_us << " us"
                << " | Min: " << min_latency << " us"
                << " | Max: " << max_latency << " us"
                << " | Avg: " << avg_latency << " us"
                << " | StdDev: " << stddev_latency << " us"
                << '\n';
        }
    }

    return 0;
}