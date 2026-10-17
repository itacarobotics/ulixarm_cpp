#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <chrono>
#include <stdexcept>

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
    using Clock = std::chrono::steady_clock;

    Motor(MotorID slave_id_, MotorID master_id_,
          MotorLimitParam limit_param_, std::string joint_name_)
        : slave_id(slave_id_),
          master_id(master_id_),
          limit_param(limit_param_),
          joint_name(std::move(joint_name_)),
          q_set(false),
          dq_set(false),
          tau_set(false),
          trot_set(false),
          tmos_set(false)
    { }

    ~Motor() = default;

    MotorID get_slave_id() const { return slave_id; }
    MotorID get_master_id() const { return master_id; }
    MotorLimitParam get_limit_param() const { return limit_param; }

    // q
    double get_q() const
    {
        check_data(q_set, q_timestamp, "q");
        return q;
    }

    void set_q(double q_)
    {
        q = q_;
        q_timestamp = Clock::now();
        q_set = true;
    }

    // dq
    double get_dq() const
    {
        check_data(dq_set, dq_timestamp, "dq");
        return dq;
    }

    void set_dq(double dq_)
    {
        dq = dq_;
        dq_timestamp = Clock::now();
        dq_set = true;
    }

    // tau
    double get_tau() const
    {
        check_data(tau_set, tau_timestamp, "tau");
        return tau;
    }

    void set_tau(double tau_)
    {
        tau = tau_;
        tau_timestamp = Clock::now();
        tau_set = true;
    }

    // trot
    int get_trot() const
    {
        check_data(trot_set, trot_timestamp, "trot");
        return trot;
    }

    void set_trot(int trot_)
    {
        trot = trot_;
        trot_timestamp = Clock::now();
        trot_set = true;
    }

    // tmos
    int get_tmos() const
    {
        check_data(tmos_set, tmos_timestamp, "tmos");
        return tmos;
    }

    void set_tmos(int tmos_)
    {
        tmos = tmos_;
        tmos_timestamp = Clock::now();
        tmos_set = true;
    }

    std::string get_joint_name() const { return joint_name; }

private:
    static constexpr auto MAX_DATA_AGE = std::chrono::milliseconds(100);

    void check_data(bool is_set,
                    const Clock::time_point& timestamp,
                    const char* variable_name) const
    {
        // Ensure the value has been set at least once.
        if (!is_set)
        {
            throw std::runtime_error(
                std::string("Motor data '") + variable_name +
                "' has not been set for joint '" + joint_name + "'");
        }

        // Check whether the last update is older than 100 ms.
        if (Clock::now() - timestamp > MAX_DATA_AGE)
        {
            throw std::runtime_error(
                std::string("Motor data '") + variable_name +
                "' is older than 100 ms for joint '" + joint_name + "'");
        }
    }

private:
    MotorID slave_id;
    MotorID master_id;
    MotorLimitParam limit_param;

    std::string joint_name;

    double q;
    double dq;
    double tau;
    int trot;
    int tmos;

    // Timestamps are initialized only when the corresponding setter is called.
    Clock::time_point q_timestamp;
    Clock::time_point dq_timestamp;
    Clock::time_point tau_timestamp;
    Clock::time_point trot_timestamp;
    Clock::time_point tmos_timestamp;

    // Flags indicate whether each value has been set at least once.
    bool q_set;
    bool dq_set;
    bool tau_set;
    bool trot_set;
    bool tmos_set;
};