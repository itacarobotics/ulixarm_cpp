// Gravity compensation with keyboard gripper control, then a stiff hold with a
// torque/temperature monitor.
//
// Phase 1 (arm floats, can be moved by hand):
//   o = open gripper, c = close gripper, q = hold the current pose and go to phase 2
// Phase 2 (arm in a stiff hold, shows torques and temperatures):
//   q = quit (the arm stays in the hold)
// Ctrl+C at any time: hold the current pose and quit.
//
// Usage: examples_gravity [port]   (default /dev/ttyACM0)

#include <ulixarm_cpp/ulixarm.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>

using ulixarm::Joints;

namespace
{
std::atomic<bool> interrupted{false};
void on_sigint(int) { interrupted = true; }

// Puts stdin in non-blocking, single-character mode and restores it on destruction.
class RawTerminal
{
public:
    RawTerminal()
    {
        tcgetattr(STDIN_FILENO, &old_termios_);
        termios raw = old_termios_;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK);
    }

    ~RawTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
        fcntl(STDIN_FILENO, F_SETFL, old_flags_);
    }

    RawTerminal(const RawTerminal&) = delete;
    RawTerminal& operator=(const RawTerminal&) = delete;

    // Returns the pressed key, or 0 if none.
    char poll() const
    {
        char c = 0;
        return read(STDIN_FILENO, &c, 1) > 0 ? c : 0;
    }

private:
    termios old_termios_{};
    int old_flags_ = 0;
};
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

    const Joints kd = Joints::Constant(0.2);  // light damping so the arm does not drift
    const auto period = std::chrono::milliseconds(2);  // 500 Hz

    RawTerminal terminal;

    try
    {
        // Phase 1: gravity compensation, gripper on the keyboard
        std::cout << "Gravity compensation active. o = open, c = close, q = hold." << std::endl;
        auto next = std::chrono::steady_clock::now();
        bool floating = true;
        while (floating && !interrupted)
        {
            // kp = 0: the motor ignores q_target, only gravity + damping act
            arm.control(Joints::Zero(), Joints::Zero(), Joints::Zero(), Joints::Zero(), kd, /*gravity=*/true);

            switch (terminal.poll())
            {
                case 'o': arm.open(); break;
                case 'c': arm.close(0.4); break;
                case 'q': floating = false; break;
                default: break;
            }

            next += period;
            std::this_thread::sleep_until(next);
        }

        arm.hold();

        // Phase 2: stiff hold, show torques and temperatures
        while (!interrupted && terminal.poll() != 'q')
        {
            arm.refresh();

            std::cout << "\033[2J\033[H";  // clear screen
            std::cout << "========== ROBOT STATE (holding, q = quit) ==========\n";
            std::cout << std::fixed << std::setprecision(2);
            std::cout << " Joint | Torque (Nm) |    TMOS (C) |    TROT (C)\n";
            std::cout << "------------------------------------------------\n";

            const Joints tau = arm.get_tau();
            const Joints tmos = arm.get_tmos();
            const Joints trot = arm.get_trot();
            for (int i = 0; i < 6; ++i)
            {
                std::cout << std::setw(6) << i + 1
                          << " | " << std::setw(11) << tau(i)
                          << " | " << std::setw(11) << tmos(i)
                          << " | " << std::setw(11) << trot(i) << '\n';
            }
            std::cout << "================================================" << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
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
