#pragma once

#include <cstdint>
#include <string>

using MotorID = uint32_t;

struct MotorLimitParam
{
    double q;
    double dq;
    double tau;
};

class Motor
{
public:
    Motor(MotorID slave_id_, MotorID master_id_, MotorLimitParam limit_param_, std::string joint_name_)
        : slave_id(slave_id_),
          master_id(master_id_),
          limit_param(limit_param_),
          joint_name(joint_name_)
    { }

    ~Motor() { }

    MotorID get_slave_id() const { return this->slave_id; };
    MotorID get_master_id() const { return this->master_id; };
    MotorLimitParam get_limit_param() const { return this->limit_param; };

    double get_q() const { return this->q; };
    void set_q(double q) { this->q = q; };
    
    double get_dq() const { return this->dq; };
    void set_dq(double dq) { this->dq = dq; };
    
    double get_tau() const { return this->tau; };
    void set_tau(double tau) { this->tau = tau; };
    
    int get_trot() const { return this->trot; };
    void set_trot(int trot) { this->trot = trot; };
    
    int get_tmos() const { return this->tmos; };
    void set_tmos(int tmos) { this->tmos = tmos; };

    std::string get_joint_name() const { return this->joint_name; };

private:
    MotorID slave_id;
    MotorID master_id;
    MotorLimitParam limit_param;    

    std::string joint_name;
    double q{};
    double dq{};
    double tau{};
    int trot{};
    int tmos{};
};