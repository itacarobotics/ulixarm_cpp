// Gravity compensation on two arms: both float and can be moved by hand.
// The loop commands arm1 then arm2 as fast as the buses allow and prints the
// control frequency and joint positions of both arms once per second.
// Ctrl+C: hold the current pose of both arms and quit.
//
// Usage: examples_gravity2 [port1] [port2]   (default /dev/ttyACM0 /dev/ttyACM1)

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

void print_q(const char* name, const Joints& q)
{
    std::printf(" | %s q [rad]:", name);
    for (int i = 0; i < q.size(); ++i)
        std::printf(" %+6.3f", q(i));
}
}

int main(int argc, char** argv)
{
    const char* port1 = argc > 1 ? argv[1] : "/dev/ttyACM0";
    const char* port2 = argc > 2 ? argv[2] : "/dev/ttyACM1";
    std::signal(SIGINT, on_sigint);

    ulixarm::UlixArm arm1(/*gripper=*/true, /*camera=*/false);
    ulixarm::UlixArm arm2(/*gripper=*/true, /*camera=*/false);

    try
    {
        arm1.connect(port1);
    }
    catch (const std::exception& e)
    {
        arm1.log_errors(e);
        return 1;
    }

    try
    {
        arm2.connect(port2);
    }
    catch (const std::exception& e)
    {
        arm2.log_errors(e);
        arm1.disconnect();
        return 1;
    }

    try
    {
        arm1.enable();
        arm2.enable();
    }
    catch (const std::exception& e)
    {
        arm1.log_errors(e);
        arm2.log_errors(e);
        arm1.disconnect();
        arm2.disconnect();
        return 1;
    }

    const Joints kp = Joints::Zero();
    const Joints kd = Joints::Constant(0.0);  // light damping so the arm does not drift

    using Clock = std::chrono::steady_clock;
    const auto print_period = std::chrono::seconds(1);

    try
    {
        std::printf("Gravity compensation active on arm1 (%s) and arm2 (%s). "
                    "Ctrl+C to hold and quit.\n", port1, port2);

        auto window_start = Clock::now();
        long cycles = 0;

        while (!interrupted)
        {
            // kp = 0: the motors ignore q_target, only gravity + damping act
            arm1.control(Joints::Zero(), Joints::Zero(), Joints::Zero(), kp, kd, /*gravity=*/true);
            arm2.control(Joints::Zero(), Joints::Zero(), Joints::Zero(), kp, kd, /*gravity=*/true);
            ++cycles;

            const auto now = Clock::now();
            const std::chrono::duration<double> elapsed = now - window_start;
            if (elapsed >= print_period)
            {
                const double freq = cycles / elapsed.count();

                std::printf("%7.1f Hz", freq);
                print_q("arm1", arm1.get_q());  // updated by control(), no refresh() needed
                print_q("arm2", arm2.get_q());
                std::printf("\n");

                window_start = now;
                cycles = 0;
            }
        }

        std::printf("\nCtrl+C received, holding the current pose of both arms.\n");
        arm1.hold();
        arm2.hold();
    }
    catch (const std::exception& e)
    {
        // no automatic fault reaction: stop commanding and leave it to the user
        arm1.log_errors(e);
        arm2.log_errors(e);
        return 1;
    }

    // both arms stay enabled in the last hold()
    arm1.disconnect();
    arm2.disconnect();
    return 0;
}
