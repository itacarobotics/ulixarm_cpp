#include "ulixarm_cpp/ulixarm.hpp"

#include "motor.hpp"
#include "protocol.hpp"
#include "serial_port.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ulixarm
{

namespace
{

// Hard-coded on purpose: these are safety limits, not user settings.
constexpr std::chrono::milliseconds max_motors_update_age{500};
constexpr double max_q_step = 0.2;              // [rad], for joints with kp > 0
constexpr int max_motor_temperature = 90;       // [C], MOS and rotor
constexpr int warn_motor_temperature = 70;      // [C], MOS and rotor
constexpr double max_gripper_opening = 0.090;   // [m]

constexpr unsigned int baudrate = 921600;       // [bit/s]
constexpr unsigned int byte_timeout_ms = 10;

// bridge status frames (heartbeat, send ack) and frames from other motors are skipped
constexpr std::size_t max_skipped_frames = 8;

std::string resolve_urdf_path(const std::string& urdf_path)
{
    if (!urdf_path.empty())
        return urdf_path;

    if (const char* env = std::getenv("ULIXARM_DESCRIPTION_PATH"))
        return std::string(env) + "/urdf/robot.urdf";

    for (const char* dir : {ULIXARM_INSTALL_DESCRIPTION_PATH, ULIXARM_SOURCE_DESCRIPTION_PATH})
    {
        const std::string path = std::string(dir) + "/urdf/robot.urdf";
        if (std::filesystem::exists(path))
            return path;
    }

    throw std::invalid_argument(
        "Could not find robot.urdf. Pass urdf_path or set ULIXARM_DESCRIPTION_PATH."
    );
}

// Inverse kinematics of the gripper: finger opening [m] -> motor angle [rad].
double gripper_ik(double d)
{
    constexpr double l1 = 0.033;    // rotor link
    constexpr double l2 = 0.035;    // rod link
    constexpr double e  = -0.0156;  // offset perpendicular to rail axis
    constexpr double f  = 0.0165;   // offset parallel to rail axis

    const double A = d + f;
    const double R = std::hypot(A, e);
    const double K = (A * A + e * e + l1 * l1 - l2 * l2) / (2.0 * l1);

    // Protect acos() against floating-point roundoff.
    const double Q = std::clamp(K / R, -1.0, 1.0);

    // Select the solution branch satisfying -1.5 <= th1 <= 0.
    return std::atan2(e, A) - std::acos(Q);
}

} // namespace


enum class Status
{
    Idle = 0,
    Connected,
    Enabled,
    Disabled,
    Running,
    Error
};


struct UlixArm::Impl
{
    bool gripper;
    Status status{Status::Idle};

    std::vector<Motor> motors_manipulator;
    std::vector<Motor> motors_gripper;

    DamiaoProtocol protocol;
    SerialPort port;

    pinocchio::Model pin_model;
    pinocchio::Data pin_data;

    // configuration and indices of the manipulator joints in pin_model, cached for the control loop
    Eigen::VectorXd q_full;
    std::array<int, kNumJoints> idx_q{};
    std::array<int, kNumJoints> idx_v{};
    int gripper_idx_q{-1};

    std::chrono::steady_clock::time_point motors_update_timestamp;
    bool temperature_warned{false};


    const pinocchio::JointModel& find_joint(const std::string& name) const
    {
        const pinocchio::JointIndex id = this->pin_model.getJointId(name);

        if (id == 0 || id >= static_cast<pinocchio::JointIndex>(this->pin_model.njoints))
        {
            throw std::out_of_range("Joint '" + name + "' not found in Pinocchio model.");
        }
        return this->pin_model.joints[id];
    }


    void require_status(std::initializer_list<Status> allowed) const
    {
        if (std::find(allowed.begin(), allowed.end(), this->status) != allowed.end())
            return;

        switch (this->status)
        {
        case Status::Idle:
            throw std::runtime_error("Robot is not connected.");
        case Status::Connected:
        case Status::Disabled:
            throw std::runtime_error("Robot is not enabled.");
        case Status::Enabled:
        case Status::Running:
            throw std::runtime_error("Robot already connected.");
        case Status::Error:
        default:
            throw std::runtime_error("Robot status error.");
        }
    }


    void check_gripper_mounted() const
    {
        if (!this->gripper || this->motors_gripper.empty())
            throw std::runtime_error("No gripper mounted.");
    }


    void set_motors_enabled(const std::vector<Motor>& motors, bool enable)
    {
        for (const auto& motor : motors)
        {
            try
            {
                const SendFrame frame = enable
                    ? this->protocol.encode_enable_command(motor.get_slave_id())
                    : this->protocol.encode_disable_command(motor.get_slave_id());

                this->port.write(frame.data, sizeof(frame.data));
            }
            catch (...)
            {
                std::throw_with_nested(std::runtime_error(
                    std::string(enable ? "Failed to enable motor: " : "Failed to disable motor: ")
                    + motor.get_joint_name()
                ));
            }
        }
    }


    /**
     * Writes frames[i] to motors[i] for all i in one write, then reads one
     * feedback per motor, matched by master id, and updates the motor state.
     */
    void transact(Motor* motors, const SendFrame* frames, std::size_t n)
    {
        if (n == 0)
            return;

        // drop stale frames once per cycle, replies are matched by id
        this->port.flush_input();
        this->port.write(frames[0].data, n * sizeof(SendFrame));

        const bool expect_enabled = this->status == Status::Enabled;
        uint32_t pending = (1u << n) - 1;

        for (std::size_t reads = 0; pending != 0 && reads < n + max_skipped_frames; ++reads)
        {
            ReceiveFrame rx;
            try
            {
                this->port.read(rx.data, DamiaoProtocol::head, DamiaoProtocol::tail, sizeof(rx.data));
            }
            catch (...)
            {
                std::throw_with_nested(std::runtime_error(
                    "Could not read feedback of motor " + first_pending(motors, n, pending)
                ));
            }

            MotorID master_id;
            Payload payload;
            if (!this->protocol.parse_usb_bridge_frame(rx, master_id, payload))
                continue;

            for (std::size_t i = 0; i < n; ++i)
            {
                if (!(pending & (1u << i)) || motors[i].get_master_id() != master_id)
                    continue;

                this->apply_feedback(motors[i], payload, expect_enabled);
                pending &= ~(1u << i);
                break;
            }
        }

        if (pending != 0)
        {
            throw std::runtime_error(
                "No feedback received from motor " + first_pending(motors, n, pending)
            );
        }
    }


    static std::string first_pending(const Motor* motors, std::size_t n, uint32_t pending)
    {
        for (std::size_t i = 0; i < n; ++i)
        {
            if (pending & (1u << i))
                return motors[i].get_joint_name();
        }
        return "";
    }


    void apply_feedback(Motor& motor, const Payload& payload, bool expect_enabled)
    {
        if (DamiaoProtocol::feedback_slave_id(payload) != (motor.get_slave_id() & 0x0F))
        {
            throw std::runtime_error(
                "Feedback of motor " + motor.get_joint_name() + " carries a different slave id."
            );
        }

        try
        {
            this->protocol.check_motor_state(payload, expect_enabled);
        }
        catch (...)
        {
            std::throw_with_nested(std::runtime_error("Motor fault on " + motor.get_joint_name()));
        }

        double q, dq, tau;
        int trot, tmos;
        this->protocol.decode_control_feedback(payload, motor.get_limit_param(), q, dq, tau, trot, tmos);

        motor.set_q(q);
        motor.set_dq(dq);
        motor.set_tau(tau);
        motor.set_trot(trot);
        motor.set_tmos(tmos);
    }


    uint32_t read_register(const Motor& motor, MotorRegister rid)
    {
        const SendFrame frame = this->protocol.encode_read_register_command(motor.get_slave_id(), rid);

        this->port.flush_input();
        this->port.write(frame.data, sizeof(frame.data));

        for (std::size_t i = 0; i <= max_skipped_frames; ++i)
        {
            ReceiveFrame rx;
            this->port.read(rx.data, DamiaoProtocol::head, DamiaoProtocol::tail, sizeof(rx.data));

            MotorID master_id;
            Payload payload;
            uint32_t raw;
            if (this->protocol.parse_usb_bridge_frame(rx, master_id, payload) &&
                this->protocol.decode_register_reply(payload, motor.get_slave_id(), rid, raw))
                return raw;
        }

        throw std::runtime_error("No reply to register read " + std::to_string(static_cast<int>(rid)) + ".");
    }


    void check_register_float(const Motor& motor, MotorRegister rid, const char* name, double expected)
    {
        const uint32_t raw = this->read_register(motor, rid);
        float value;
        std::memcpy(&value, &raw, sizeof(value));

        if (!(std::abs(value - expected) <= 1e-3 * std::abs(expected)))
        {
            throw std::runtime_error(
                std::string(name) + " is " + std::to_string(value) +
                ", expected " + std::to_string(expected) + "."
            );
        }
    }


    // shortcut: compares PMAX/VMAX/TMAX against the hard-coded values, upgrade to reading them into limit_param if motors vary
    void verify_motor_config()
    {
        for (const auto* motors : {&this->motors_manipulator, &this->motors_gripper})
        {
            for (const auto& motor : *motors)
            {
                try
                {
                    const uint32_t mode = this->read_register(motor, MotorRegister::CTRL_MODE);
                    if (mode != ctrl_mode_mit)
                    {
                        throw std::runtime_error(
                            "Motor is not in MIT mode (control mode " + std::to_string(mode) + ")."
                        );
                    }

                    const MotorLimitParam& limit = motor.get_limit_param();
                    this->check_register_float(motor, MotorRegister::PMAX, "PMAX", limit.q);
                    this->check_register_float(motor, MotorRegister::VMAX, "VMAX", limit.dq);
                    this->check_register_float(motor, MotorRegister::TMAX, "TMAX", limit.tau);
                }
                catch (...)
                {
                    std::throw_with_nested(std::runtime_error(
                        "Invalid configuration of motor " + motor.get_joint_name()
                    ));
                }
            }
        }
    }


    void refresh_motors(std::vector<Motor>& motors)
    {
        std::array<SendFrame, kNumJoints> frames;
        for (std::size_t i = 0; i < motors.size(); ++i)
            frames[i] = this->protocol.encode_refresh_command(motors[i].get_slave_id());

        this->transact(motors.data(), frames.data(), motors.size());
    }


    void check_fresh() const
    {
        if (std::chrono::steady_clock::now() - this->motors_update_timestamp > max_motors_update_age)
        {
            throw std::runtime_error(
                "Cannot consume motor data older than " +
                std::to_string(max_motors_update_age.count()) + " ms."
            );
        }
    }


    template <typename Get>
    Joints collect(Get get) const
    {
        this->check_fresh();

        Joints out;
        for (std::size_t i = 0; i < kNumJoints; ++i)
            out[i] = get(this->motors_manipulator[i]);
        return out;
    }


    Joints get_q() const
    {
        return this->collect([](const Motor& m) { return m.get_q(); });
    }


    Joints compute_tau_gravity(const Joints& q)
    {
        for (std::size_t i = 0; i < kNumJoints; ++i)
            this->q_full[this->idx_q[i]] = q[i];

        const Eigen::VectorXd& g =
            pinocchio::computeGeneralizedGravity(this->pin_model, this->pin_data, this->q_full);

        Joints tau;
        for (std::size_t i = 0; i < kNumJoints; ++i)
            tau[i] = g[this->idx_v[i]];
        return tau;
    }


    void check_manipulator_joint_limits(const Joints& q_target, const Joints& dq_target,
        const Joints& tau_target, const Joints& kp, const Joints& kd) const
    {
        for (std::size_t i = 0; i < kNumJoints; ++i)
        {
            const auto& motor = this->motors_manipulator[i];
            const MotorLimitParam& limit_param = motor.get_limit_param();

            // negated comparisons so that NaN fails the checks
            const double q = q_target[i];
            if (!(q >= this->pin_model.lowerPositionLimit[this->idx_q[i]] &&
                  q <= this->pin_model.upperPositionLimit[this->idx_q[i]]))
            {
                throw std::out_of_range("Joint '" + motor.get_joint_name() + "' position limit exceeded.");
            }

            // Effective velocity limit: minimum of URDF and configured limits
            const double max_dq = std::min(this->pin_model.velocityLimit[this->idx_v[i]], limit_param.dq);
            if (!(std::abs(dq_target[i]) <= max_dq))
            {
                throw std::out_of_range("Joint '" + motor.get_joint_name() + "' velocity limit exceeded.");
            }

            // Effective effort limit: minimum of URDF and configured limits
            const double max_tau = std::min(this->pin_model.effortLimit[this->idx_v[i]], limit_param.tau);
            if (!(std::abs(tau_target[i]) <= max_tau))
            {
                throw std::out_of_range("Joint '" + motor.get_joint_name() + "' effort limit exceeded.");
            }

            // Gains: range supported by the motor protocol
            if (!(kp[i] >= 0.0 && kp[i] <= DamiaoProtocol::kp_max))
            {
                throw std::out_of_range("Joint '" + motor.get_joint_name() + "' kp out of range [0, 500].");
            }

            if (!(kd[i] >= 0.0 && kd[i] <= DamiaoProtocol::kd_max))
            {
                throw std::out_of_range("Joint '" + motor.get_joint_name() + "' kd out of range [0, 5].");
            }
        }
    }


    void check_position_step(const Joints& q_target, const Joints& kp) const
    {
        // no stiffness -> target position has no effect
        if ((kp.array() <= 0.0).all())
            return;

        const Joints q = this->get_q();

        for (std::size_t i = 0; i < kNumJoints; ++i)
        {
            if (kp[i] > 0.0 && std::abs(q_target[i] - q[i]) > max_q_step)
            {
                throw std::out_of_range(
                    "Joint '" + this->motors_manipulator[i].get_joint_name() +
                    "' target is " + std::to_string(std::abs(q_target[i] - q[i])) +
                    " rad away from current position (max " + std::to_string(max_q_step) +
                    " rad). Interpolate the motion."
                );
            }
        }
    }


    void check_motor_temperatures()
    {
        bool any_warm = false;

        for (const auto* motors : {&this->motors_manipulator, &this->motors_gripper})
        {
            for (const auto& motor : *motors)
            {
                const int t = std::max(motor.get_tmos(), motor.get_trot());

                if (t > max_motor_temperature)
                {
                    throw std::runtime_error(
                        "Motor '" + motor.get_joint_name() + "' overheating: MOS " +
                        std::to_string(motor.get_tmos()) + " C, rotor " +
                        std::to_string(motor.get_trot()) + " C (max " +
                        std::to_string(max_motor_temperature) + " C)."
                    );
                }

                if (t > warn_motor_temperature)
                {
                    any_warm = true;

                    // print once until all motors cool down, to keep the control loop quiet
                    if (!this->temperature_warned)
                    {
                        std::cerr << "Warning: motor '" << motor.get_joint_name() << "' is hot: MOS "
                                  << motor.get_tmos() << " C, rotor " << motor.get_trot()
                                  << " C (error at " << max_motor_temperature << " C).\n";
                        this->temperature_warned = true;
                    }
                }
            }
        }

        if (!any_warm)
            this->temperature_warned = false;
    }


    void check_gripper_joint_limits(double d_target) const
    {
        if (!(d_target >= this->pin_model.lowerPositionLimit[this->gripper_idx_q] &&
              d_target <= this->pin_model.upperPositionLimit[this->gripper_idx_q]))
        {
            throw std::out_of_range(
                "Joint '" + this->motors_gripper[0].get_joint_name() + "' position limit exceeded."
            );
        }
    }


    void command_gripper(double q, double tau, double kp, double kd)
    {
        auto& motor = this->motors_gripper[0];

        try
        {
            const SendFrame frame = this->protocol.encode_control_command(
                motor.get_slave_id(), motor.get_limit_param(), q, 0.0, tau, kp, kd
            );
            this->transact(&motor, &frame, 1);
        }
        catch (...)
        {
            std::throw_with_nested(std::runtime_error("Failed to control motor " + motor.get_joint_name()));
        }
    }
};


UlixArm::UlixArm(bool gripper, const std::string& urdf_path)
    : impl(std::make_unique<Impl>())
{
    auto& d = *this->impl;
    d.gripper = gripper;

    if (d.gripper)
    {
        d.motors_gripper.emplace_back(0x07, 0x17, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}, "finger_right_joint"); // DM4310
    }

    d.motors_manipulator.emplace_back(0x01, 0x11, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}, "joint1"); // DM4340p
    d.motors_manipulator.emplace_back(0x02, 0x12, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}, "joint2"); // DM4340
    d.motors_manipulator.emplace_back(0x03, 0x13, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}, "joint3"); // DM4340
    d.motors_manipulator.emplace_back(0x04, 0x14, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}, "joint4"); // DM4310
    d.motors_manipulator.emplace_back(0x05, 0x15, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}, "joint5"); // DM4310
    d.motors_manipulator.emplace_back(0x06, 0x16, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}, "joint6"); // DM4310

    const std::string model_path = resolve_urdf_path(urdf_path);

    try
    {
        pinocchio::urdf::buildModel(model_path, d.pin_model);
        d.pin_data = pinocchio::Data(d.pin_model);
    }
    catch (...)
    {
        std::throw_with_nested(std::invalid_argument("Failed to load URDF '" + model_path + "'."));
    }

    // urdf might have more joints than motors (two fingers), they stay at their neutral position
    d.q_full = pinocchio::neutral(d.pin_model);

    for (std::size_t i = 0; i < kNumJoints; ++i)
    {
        const auto& joint = d.find_joint(d.motors_manipulator[i].get_joint_name());
        d.idx_q[i] = joint.idx_q();
        d.idx_v[i] = joint.idx_v();
    }

    if (d.gripper)
    {
        d.gripper_idx_q = d.find_joint(d.motors_gripper[0].get_joint_name()).idx_q();
    }
}


// serial port is closed by its destructor, motors stay enabled to avoid the arm falling
UlixArm::~UlixArm() = default;


void UlixArm::connect(const char* port)
{
    auto& d = *this->impl;

    if (d.status == Status::Error)
        throw std::runtime_error("Robot status error.");
    if (d.status != Status::Idle)
        throw std::runtime_error("Robot already connected.");

    try
    {
        d.port.init(port, baudrate, byte_timeout_ms);
    }
    catch (...)
    {
        std::throw_with_nested(std::runtime_error("Failed to init port."));
    }

    try
    {
        d.verify_motor_config();
    }
    catch (...)
    {
        d.port.close();
        std::throw_with_nested(std::runtime_error("Motor configuration check failed."));
    }

    d.status = Status::Connected;
}


void UlixArm::disconnect()
{
    this->impl->port.close();
    this->impl->status = Status::Idle;
}


void UlixArm::enable()
{
    auto& d = *this->impl;
    d.require_status({Status::Connected, Status::Disabled, Status::Enabled});

    d.set_motors_enabled(d.motors_manipulator, true);
    d.set_motors_enabled(d.motors_gripper, true);

    d.status = Status::Enabled;

    // also checks that every motor now reports enabled
    this->refresh();
}


void UlixArm::disable()
{
    auto& d = *this->impl;
    d.require_status({Status::Connected, Status::Disabled, Status::Enabled});

    d.set_motors_enabled(d.motors_manipulator, false);
    d.set_motors_enabled(d.motors_gripper, false);

    d.status = Status::Disabled;
}


void UlixArm::refresh()
{
    auto& d = *this->impl;
    d.require_status({Status::Connected, Status::Disabled, Status::Enabled});

    try
    {
        d.refresh_motors(d.motors_manipulator);
        d.refresh_motors(d.motors_gripper);
    }
    catch (...)
    {
        std::throw_with_nested(std::runtime_error("Failed to refresh motors."));
    }

    d.motors_update_timestamp = std::chrono::steady_clock::now();
}


void UlixArm::control(
    const Joints& q_target,
    const Joints& dq_target,
    const Joints& tau_target,
    const Joints& kp,
    const Joints& kd,
    bool gravity
)
{
    auto& d = *this->impl;
    d.require_status({Status::Enabled});

    Joints tau = tau_target;
    if (gravity)
        tau += d.compute_tau_gravity(d.get_q());

    try
    {
        d.check_manipulator_joint_limits(q_target, dq_target, tau, kp, kd);
        d.check_position_step(q_target, kp);
        d.check_motor_temperatures();
    }
    catch (...)
    {
        std::throw_with_nested(std::runtime_error("Invalid target command."));
    }

    std::array<SendFrame, kNumJoints> frames;

    try
    {
        for (std::size_t i = 0; i < kNumJoints; ++i)
        {
            const auto& motor = d.motors_manipulator[i];
            frames[i] = d.protocol.encode_control_command(
                motor.get_slave_id(), motor.get_limit_param(),
                q_target[i], dq_target[i], tau[i], kp[i], kd[i]
            );
        }

        d.transact(d.motors_manipulator.data(), frames.data(), kNumJoints);
    }
    catch (...)
    {
        std::throw_with_nested(std::runtime_error("Failed to send motor commands."));
    }

    d.motors_update_timestamp = std::chrono::steady_clock::now();
}


void UlixArm::hold()
{
    Joints kp_hold;
    kp_hold << 50.0, 100.0, 60.0, 30.0, 20.0, 10.0;

    Joints kd_hold;
    kd_hold << 1.0, 2.0, 1.5, 0.8, 0.5, 0.2;

    // update motor status to latest position
    this->refresh();

    this->control(this->get_q(), Joints::Zero(), Joints::Zero(), kp_hold, kd_hold, true);
}


Joints UlixArm::get_q() const
{
    return this->impl->get_q();
}

Joints UlixArm::get_dq() const
{
    return this->impl->collect([](const Motor& m) { return m.get_dq(); });
}

Joints UlixArm::get_tau() const
{
    return this->impl->collect([](const Motor& m) { return m.get_tau(); });
}

Joints UlixArm::get_tmos() const
{
    return this->impl->collect([](const Motor& m) { return static_cast<double>(m.get_tmos()); });
}

Joints UlixArm::get_trot() const
{
    return this->impl->collect([](const Motor& m) { return static_cast<double>(m.get_trot()); });
}


void UlixArm::open()
{
    this->open_ratio(0.8);
}


void UlixArm::open_ratio(double ratio)
{
    this->open_width(ratio * max_gripper_opening);
}


void UlixArm::open_width(double meters)
{
    auto& d = *this->impl;
    d.check_gripper_mounted();
    d.require_status({Status::Enabled});

    // finger joint moves half the opening
    const double d_half = meters / 2.0;
    d.check_gripper_joint_limits(d_half);

    // motor angle is zero for opening zero
    const double q_open = gripper_ik(0.0) - gripper_ik(d_half);

    d.command_gripper(q_open, 0.0, 4.0, 0.5);
}


void UlixArm::close()
{
    this->close(0.5);   // default: close with half force
}


void UlixArm::close(double f)
{
    auto& d = *this->impl;
    d.check_gripper_mounted();
    d.require_status({Status::Enabled});

    // negated comparison so that NaN fails the check
    if (!(f >= 0.0 && f <= 1.0))
    {
        throw std::out_of_range(
            "Force factor limit exceeded. Accepted range [0,1], got " + std::to_string(f)
        );
    }

    constexpr double tau_close_min = 0.2;   // min torque of motor to close gripper in Nm (from heuristic)
    constexpr double tau_close_max = 1.2;   // max torque of motor to not break fingers in Nm (from heuristic)
    const double tau_close = tau_close_min + f * (tau_close_max - tau_close_min);

    d.command_gripper(0.0, tau_close, 0.0, 0.1);
}


void UlixArm::log_errors(const std::exception& e, int level)
{
    std::cerr << std::string(level * 2, ' ')
              << (level == 0 ? "Error: " : "Caused by: ")
              << e.what() << '\n';

    try
    {
        std::rethrow_if_nested(e);
    }
    catch (const std::exception& nested)
    {
        log_errors(nested, level + 1);
    }
    catch (...)
    {
        std::cerr << std::string((level + 1) * 2, ' ') << "Caused by: Unknown exception\n";
    }
}

} // namespace ulixarm
