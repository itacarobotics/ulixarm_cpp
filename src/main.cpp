#include "ulixarm.hpp"

int main()
{
    UlixArm robot(/*gripper*/ false);

    robot.connect();
    robot.enable();

    joint_t q;
    q << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7;

    joint_t dq;
    dq << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;

    joint_t tau;
    tau << -1.1, -2.2, -3.3, -4.4, -5.5, -6.6, -7.7;

    joint_t kp;
    kp << 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0;

    joint_t kd;
    kd << 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07;

    robot.control(q, dq, tau, kp, kd);

    robot.disable();

}