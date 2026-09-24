#ifndef ULIXARM_CPP_ULIXARM_HPP
#define ULIXARM_CPP_ULIXARM_HPP

#include <iostream>
#include <Eigen/Dense>
#include <stdint.h>
#include <vector>

#include "motor.hpp"
#include "protocol.hpp"
#include "serial_port.hpp"


#define N_JOINTS 7

typedef Eigen::Matrix<double, N_JOINTS, 1> Joints;

enum UlixArmStatus
{
    IDLE=0,
    CONNECTED,
    ENABLED,
    DISABLED,
    RUNNING,
    ERROR
};

class UlixArm
{
public:
    UlixArm(bool gripper=true);
    ~UlixArm();

    Joints get_q();
    Joints get_dq();
    Joints get_tau();
    Joints get_tmos();
    Joints get_trot();

    uint32_t control(
        Joints q_target,
        Joints dq_target,
        Joints tau_target,
        Joints kp,
        Joints kd,
        bool gravity=true
    );

    uint32_t enable();
    uint32_t disable();
    uint32_t connect();

private:
    UlixArmStatus status;
    std::vector<Motor> motors;

    DamiaoProtocol protocol;
    SerialPort port;
};


#endif // ULIXARM_CPP_ULIXARM_HPP