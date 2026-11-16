// Gravity compensation: the arm floats and can be moved by hand.
// The loop runs as fast as the bus allows and prints the control frequency
// and joint positions once per second.
// Ctrl+C: hold the current pose and quit.
//
// Usage: examples_gravity [port]   (default /dev/ttyACM0)

#include <ulixarm_cpp/ulixarm.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>

using ulixarm::Joints;

namespace
{
std::atomic<bool> interrupted{false};
void on_sigint(int) { interrupted = true; }
}

int main(int argc, char** argv)
{
    const char* port = argc > 1 ? argv[1] : "/dev/ttyACM0";
    std::signal(SIGINT, on_sigint);

    ulixarm::UlixArm arm(/*gripper=*/true);
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

    const Joints kp = Joints::Zero();
    const Joints kd = Joints::Constant(0.0);  // light damping so the arm does not drift

    using Clock = std::chrono::steady_clock;
    const auto print_period = std::chrono::seconds(1);

    try
    {
        std::printf("Gravity compensation active. Ctrl+C to hold and quit.\n");

        auto window_start = Clock::now();
        long cycles = 0;

        while (!interrupted)
        {
            // kp = 0: the motor ignores q_target, only gravity + damping act
            arm.control(Joints::Zero(), Joints::Zero(), Joints::Zero(), kp, kd, /*gravity=*/true);
            ++cycles;

            const auto now = Clock::now();
            const std::chrono::duration<double> elapsed = now - window_start;
            if (elapsed >= print_period)
            {
                const double freq = cycles / elapsed.count();
                const Joints q = arm.get_q();  // updated by control(), no refresh() needed

                std::printf("%7.1f Hz | q [rad]:", freq);
                for (int i = 0; i < q.size(); ++i)
                    std::printf(" %+6.3f", q(i));
                std::printf("\n");

                window_start = now;
                cycles = 0;
            }
        }

        std::printf("\nCtrl+C received, holding the current pose.\n");
        arm.hold();
    }
    catch (const std::exception& e)
    {
        // no automatic fault reaction: stop commanding and leave it to the user
        arm.log_errors(e);
        return 1;
    }

    // the arm stays enabled in the last hold()
    arm.disconnect();
    return 0;
}
