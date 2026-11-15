#pragma once

#include <cstdint>
#include <string>
#include <utility>

namespace ulixarm
{

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
          joint_name(std::move(joint_name_))
    { }

    MotorID get_slave_id() const { return this->slave_id; }
    MotorID get_master_id() const { return this->master_id; }
    const MotorLimitParam& get_limit_param() const { return this->limit_param; }
    const std::string& get_joint_name() const { return this->joint_name; }

    double get_q() const { return this->q; }
    void set_q(double q_) { this->q = q_; }

    double get_dq() const { return this->dq; }
    void set_dq(double dq_) { this->dq = dq_; }

    double get_tau() const { return this->tau; }
    void set_tau(double tau_) { this->tau = tau_; }

    int get_trot() const { return this->trot; }
    void set_trot(int trot_) { this->trot = trot_; }

    int get_tmos() const { return this->tmos; }
    void set_tmos(int tmos_) { this->tmos = tmos_; }

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

} // namespace ulixarm
