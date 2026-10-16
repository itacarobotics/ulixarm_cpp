#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <stdint.h>
#include <vector>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include "motor.hpp"
#include "protocol.hpp"
#include "serial_port.hpp"



#define N_JOINTS 6

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

    void control(
        Joints q_target,
        Joints dq_target,
        Joints tau_target,
        Joints kp,
        Joints kd,
        bool gravity
    );

    void enable();
    void disable();
    void refresh();
    void connect(const char* port);
    void hold();

    // void open();
    // void open(double d);

    // void close();
    // void close(double f);
    // void close(double d, double k);

    void log_errors(const std::exception& e, int level = 0);


private:

    void enable_motors(const std::vector<Motor>& motors);
    void disable_motors(const std::vector<Motor>& motors);
    void refresh_motors(const std::vector<Motor>& motors);

    Motor* find_motor_by_master_id(MotorID master_id);
    void read_feedback();

    double gripper_ik(double s);
    double gripper_fk(double theta);

    Joints compute_tau_gravity(Joints q);

    void check_joint_limits(Joints q_target, Joints dq_target, Joints tau_target);

    bool gripper;

    UlixArmStatus status;
    std::vector<Motor> motors_manipulator;
    std::vector<Motor> motors_gripper;

    DamiaoProtocol protocol;
    SerialPort port;

    pinocchio::Model pin_model;
    pinocchio::Data pin_data;

    const uint baudrate = 921600;
    const uint timeout_ms = 10;

};