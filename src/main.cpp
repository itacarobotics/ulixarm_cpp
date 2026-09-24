#include "ulixarm_cpp/ulixarm.hpp"

#include <thread>
#include <chrono>

int main()
{
    UlixArm robot(/*gripper*/ false);

    robot.connect();

    for (size_t i = 0; i < 10; i++)
    {
        robot.enable();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        robot.disable();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    robot.enable();
    

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
    
    //     std::this_thread::sleep_for(std::chrono::milliseconds(2));
    
        robot.refresh();
        std::cout << robot.get_q()(0) << " " << robot.get_q()(1) << " " << robot.get_q()(2) << " " << robot.get_q()(3) << std::endl;

    }



    robot.disable();
}



