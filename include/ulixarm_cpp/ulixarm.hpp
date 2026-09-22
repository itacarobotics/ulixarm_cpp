#ifndef ULIXARM_CPP_ULIXARM_HPP
#define ULIXARM_CPP_ULIXARM_HPP

#include <iostream>
#include <Eigen/Dense>
#include <stdint.h>
#include <vector>

#include "motor.hpp"
#include "protocol.hpp"


#define N_JOINTS 7

typedef Eigen::Matrix<double, N_JOINTS, 1> joint_t;

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

    joint_t get_q();
    joint_t get_dq();
    joint_t get_tau();
    joint_t get_tmos();
    joint_t get_trot();

    uint32_t control(
        joint_t q_target,
        joint_t dq_target,
        joint_t tau_target,
        joint_t kp,
        joint_t kd
    );

    uint32_t enable();
    uint32_t disable();
    uint32_t connect();

private:
    UlixArmStatus status;
    std::vector<Motor> motors;

    DamiaoProtocol protocol;
};


#endif // ULIXARM_CPP_ULIXARM_HPP