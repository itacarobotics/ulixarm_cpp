// Gravity compensation: the arm floats and can be moved by hand.
// Ctrl+C leaves the arm in a stiff hold at the current pose.
//
// Usage: examples_gravity [port]   (default /dev/ttyACM0)

#include <ulixarm_cpp/ulixarm.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

using ulixarm::Joints;

namespace
{
std::atomic<bool> running{true};
void on_sigint(int) { running = false; }
}

int main(int argc, char** argv)
{
    const char* port = argc > 1 ? argv[1] : "/dev/ttyACM0";
    std::signal(SIGINT, on_sigint);

    ulixarm::UlixArm arm;
    try
    {
        arm.connect(port);
        arm.enable();
    }
    catch (const std::exception& e)
    {
        arm.log_errors(e);
        return 1;
    }

    const Joints kd = Joints::Constant(0.2);  // light damping so the arm does not drift
    const auto period = std::chrono::milliseconds(2);  // 500 Hz
    auto next = std::chrono::steady_clock::now();

    std::cout << "Gravity compensation active, Ctrl+C to stop." << std::endl;
    try
    {
        while (running)
        {
            // kp = 0: q_target is ignored by the motor, pass the measured pose to stay within limits
            arm.control(Joints::Zero(), Joints::Zero(), Joints::Zero(), Joints::Zero(), kd, /*gravity=*/true);
            next += period;
            std::this_thread::sleep_until(next);
        }
        arm.hold();
    }
    catch (const std::exception& e)
    {
        // no automatic fault reaction: stop commanding and leave it to the user
        arm.log_errors(e);
        return 1;
    }

    arm.disconnect();
    return 0;
}
