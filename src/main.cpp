#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include "ulixarm_cpp/ulixarm.hpp"

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
        robot.log_errors(e);
        return -1;
    }

    Joints q   = Joints::Zero();
    Joints dq  = Joints::Zero();
    Joints tau = Joints::Zero();
    Joints kp  = Joints::Zero();
    Joints kd  = Joints::Zero();

    // Configure terminal for non-blocking, single-character input
    termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);

    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);

    bool running = true;

    while (running)
    {
        // Keep robot control running
        robot.control(q, dq, tau, kp, kd, true);

        char input;
        if (read(STDIN_FILENO, &input, 1) > 0)
        {
            switch (input)
            {
                case 'o':
                    robot.open();
                    break;

                case 'c':
                    robot.close(0.4);
                    break;

                case 'q':
                    running = false;
                    break;
            }
        }
    }

    robot.hold();

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, old_flags);
    
    return 0;
}