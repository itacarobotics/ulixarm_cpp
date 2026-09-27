#include "ulixarm_cpp/ulixarm.hpp"

#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>

void print_exception(const std::exception& e, int level = 0);


int main()
{
    UlixArm robot(/*gripper*/ false);

    try
    {
        robot.connect("/dev/ttyACM0");
    }
    catch(const std::exception& e)
    {
        print_exception(e);
        return -1;
    }
    

    for (size_t i = 0; i < 10; i++)
    {
        robot.enable();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        robot.disable();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    robot.enable();

    // return 0;
    std::vector<double> timings_us;

    const double timeout = 10.0;
    
    auto start = std::chrono::steady_clock::now();
    
    while (std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count() < timeout)
    {
    //     double t = std::chrono::duration<double>(
    //         std::chrono::steady_clock::now().time_since_epoch()).count();
    
    //     double sin_val = std::sin(t);
    //     double cos_val = std::cos(t);
    
    //     Joints q;
    //     q << sin_val, sin_val, sin_val, sin_val, sin_val, sin_val, sin_val;
        
    //     Joints dq;
    //     dq << cos_val, cos_val, cos_val, cos_val, cos_val, cos_val, cos_val;
        
    //     Joints tau;
    //     tau << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        
    //     Joints kp;
    //     kp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    //     // kp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        
    //     Joints kd;
    //     kd << 2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        
    //     robot.control(q, dq, tau, kp, kd);
    
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
        auto start = std::chrono::steady_clock::now();

        robot.refresh();

        auto end = std::chrono::steady_clock::now();

        double elapsed_us =
            std::chrono::duration<double, std::micro>(end - start).count();

        timings_us.push_back(elapsed_us);

        // Calculate mean
        double mean_us = 0.0;
        for (double t : timings_us)
            mean_us += t;
        mean_us /= timings_us.size();

        // Calculate max
        double max_us = *std::max_element(timings_us.begin(), timings_us.end());

        // Calculate standard deviation
        double variance = 0.0;
        for (double t : timings_us)
            variance += (t - mean_us) * (t - mean_us);
        variance /= timings_us.size();

        double std_us = std::sqrt(variance);

        // Overwrite the same terminal line
        std::cout << "\r\033[K"
                << "robot.refresh(): mean=" << mean_us
                << " us, max=" << max_us
                << " us, std=" << std_us
                << " us"
                << std::flush;

        // robot.refresh();
        // std::cout << robot.get_q()(0) << " " << robot.get_q()(1) << " " << robot.get_q()(2) << " " << robot.get_q()(3) << std::endl;

    }

    robot.disable();
}



void print_exception(const std::exception& e, int level = 0)
{
    std::cerr << std::string(level * 2, ' ')
              << e.what() << '\n';

    try
    {
        std::rethrow_if_nested(e);
    }
    catch (const std::exception& nested)
    {
        print_exception(nested, level + 1);
    }
    catch (...)
    {
        std::cerr << std::string((level + 1) * 2, ' ')
                  << "Unknown exception\n";
    }
}