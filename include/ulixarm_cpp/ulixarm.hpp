#pragma once

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

    int control(
        Joints q_target,
        Joints dq_target,
        Joints tau_target,
        Joints kp,
        Joints kd,
        bool gravity=true
    );

    int enable();
    int disable();
    int refresh();
    void connect(const char* port);

private:
    
    Motor* find_motor_by_master_id(MotorID master_id);
    void read_feedback();

    UlixArmStatus status;
    std::vector<Motor> motors;

    DamiaoProtocol protocol;
    SerialPort port;

    const uint baudrate = 921600;
    const uint timeout_ms = 10;

};