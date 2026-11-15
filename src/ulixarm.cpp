#include "ulixarm_cpp/ulixarm.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace ulixarm
{

UlixArm::UlixArm(bool gripper, const std::string& urdf_path)
{
    // save config
    this->gripper = gripper;

    // add motors of gripper
    if (this->gripper)
    {
        this->motors_gripper.emplace_back(0x07, 0x17, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}, "finger_right_joint"); // DM4310
    }

    // add motors of manipulator
    this->motors_manipulator.emplace_back(0x01, 0x11, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}, "joint1"); // DM4340p
    this->motors_manipulator.emplace_back(0x02, 0x12, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}, "joint2"); // DM4340
    this->motors_manipulator.emplace_back(0x03, 0x13, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}, "joint3"); // DM4340
    this->motors_manipulator.emplace_back(0x04, 0x14, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}, "joint4"); // DM4310
    this->motors_manipulator.emplace_back(0x05, 0x15, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}, "joint5"); // DM4310
    this->motors_manipulator.emplace_back(0x06, 0x16, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}, "joint6"); // DM4310

    // create pinocchio model and data
    const std::string model_path = this->resolve_urdf_path(urdf_path);

    try
    {
        pinocchio::urdf::buildModel(model_path, this->pin_model);
        this->pin_data = pinocchio::Data(this->pin_model);
    }
    catch(...)
    {
        std::throw_with_nested(
            std::invalid_argument("Failed to load URDF '" + model_path + "'.")
        );
    }

    // urdf has more joints than motors (two fingers), they stay at their neutral position
    this->pin_q = pinocchio::neutral(this->pin_model);

    // look up the joints by name once, so the control loop does not have to
    for (std::size_t i = 0; i < kNumJoints; ++i)
    {
        const std::string& joint_name = this->motors_manipulator[i].get_joint_name();

        if (!this->pin_model.existJointName(joint_name))
        {
            throw std::out_of_range("Joint '" + joint_name + "' not found in Pinocchio model.");
        }

        const auto& joint = this->pin_model.joints[this->pin_model.getJointId(joint_name)];
        this->joint_idx_q[i] = joint.idx_q();
        this->joint_idx_v[i] = joint.idx_v();
    }

    if (this->gripper)
    {
        const std::string& joint_name = this->motors_gripper[0].get_joint_name();

        if (!this->pin_model.existJointName(joint_name))
        {
            throw std::out_of_range("Joint '" + joint_name + "' not found in Pinocchio model.");
        }

        this->gripper_idx_q = this->pin_model.joints[this->pin_model.getJointId(joint_name)].idx_q();
    }

    // init status
    this->status = Status::IDLE;
}

UlixArm::~UlixArm()
{
    // serial port is automatically closed from destructor.

    // keep motors enabled to avoid arm falling.
}


std::string UlixArm::resolve_urdf_path(const std::string& urdf_path)
{
    if (!urdf_path.empty())
    {
        return urdf_path;
    }

    if (const char* env = std::getenv("ULIXARM_DESCRIPTION_PATH"))
    {
        return std::string(env) + "/urdf/robot.urdf";
    }

    const std::string installed = std::string(ULIXARM_INSTALL_DESCRIPTION_PATH) + "/urdf/robot.urdf";
    if (std::filesystem::exists(installed))
    {
        return installed;
    }

    const std::string source = std::string(ULIXARM_SOURCE_DESCRIPTION_PATH) + "/urdf/robot.urdf";
    if (std::filesystem::exists(source))
    {
        return source;
    }

    throw std::invalid_argument(
        "Could not find robot.urdf. Pass urdf_path or set ULIXARM_DESCRIPTION_PATH."
    );
}


void UlixArm::connect(const char* port)
{
    // check status
    switch (this->status)
    {
    case Status::CONNECTED:
    case Status::DISABLED:
    case Status::ENABLED:
    case Status::RUNNING:
        throw std::runtime_error("Robot already connected.");
    case Status::ERROR:
        throw std::runtime_error("Robot status error.");
    case Status::IDLE:
    default:
        break;
    }

    // open bus
    try
    {
        this->port.init(
            port,       // name of port
            static_cast <unsigned int>(921600), // baudrate [bit/s]
            static_cast <unsigned int>(10)      // byte timeout [ms]
        );
    }
    catch(...)
    {
        std::throw_with_nested(
            std::runtime_error("Failed to init port.")
        );
    }

    // check that motors are configured as expected
    try
    {
        this->verify_motor_config(this->motors_manipulator);
        this->verify_motor_config(this->motors_gripper);
    }
    catch(...)
    {
        this->port.close();
        std::throw_with_nested(
            std::runtime_error("Motor configuration check failed.")
        );
    }

    this->status = Status::CONNECTED;
}


void UlixArm::disconnect()
{
    // motors are not disabled, to avoid arm falling.
    this->port.close();
    this->status = Status::IDLE;
}


void UlixArm::verify_motor_config(const std::vector<Motor>& motors)
{
    for (const auto& motor : motors) {

        try
        {
            // the library only speaks the MIT protocol
            const uint32_t mode = this->read_register(motor, MotorRegister::CTRL_MODE);

            if (mode != ctrl_mode_mit)
            {
                throw std::runtime_error(
                    "Motor is not in MIT mode (control mode " + std::to_string(mode) + ")."
                );
            }

            // the mapping ranges must match the ones used to encode and decode
            const MotorLimitParam& limit_param = motor.get_limit_param();
            this->check_register_value(motor, MotorRegister::PMAX, "PMAX", limit_param.q);
            this->check_register_value(motor, MotorRegister::VMAX, "VMAX", limit_param.dq);
            this->check_register_value(motor, MotorRegister::TMAX, "TMAX", limit_param.tau);
        }
        catch(...)
        {
            std::throw_with_nested(
                std::runtime_error("Invalid configuration of motor " + motor.get_joint_name())
            );
        }
    }
}


uint32_t UlixArm::read_register(const Motor& motor, MotorRegister rid)
{
    // send read request
    SendFrame send_frame = this->protocol.encode_read_register_command(motor.get_slave_id(), rid);
    this->send_frames(&send_frame, 1);

    // read until the reply arrives
    for (std::size_t i = 0; i <= this->max_skipped_frames; ++i)
    {
        ReceiveFrame receive_frame;
        this->port.read(
            receive_frame.data,
            this->protocol.head,
            this->protocol.tail,
            sizeof(receive_frame.data)
        );

        MotorID master_id;
        Payload payload;
        uint32_t value;

        if (this->protocol.parse_usb_bridge_frame(receive_frame, master_id, payload) &&
            this->protocol.decode_register_reply(payload, motor.get_slave_id(), rid, value))
        {
            return value;
        }
    }

    throw std::runtime_error(
        "No reply to register read " + std::to_string(static_cast<int>(rid)) + "."
    );
}


void UlixArm::check_register_value(const Motor& motor, MotorRegister rid, const std::string& name, double expected)
{
    // register holds a float
    const uint32_t raw = this->read_register(motor, rid);
    float value;
    std::memcpy(&value, &raw, sizeof(value));

    if (std::abs(value - expected) > 1e-3 * std::abs(expected))
    {
        throw std::runtime_error(
            name + " is " + std::to_string(value) +
            ", expected " + std::to_string(expected) + "."
        );
    }
}


void UlixArm::enable_motors(const std::vector<Motor>& motors)
{
    for (const auto& motor : motors) {

        try
        {
            // pack frame
            SendFrame send_frame = this->protocol.encode_enable_command(
                motor.get_slave_id()
            );

            // send frame
            this->port.write(send_frame.data, sizeof(send_frame.data));
        }
        catch(...)
        {
            std::throw_with_nested(
                std::runtime_error("Failed to enable motor: " + motor.get_joint_name())
            );
        }
    }
}

void UlixArm::enable()
{
    // check status
    switch (this->status)
    {
    case Status::IDLE:
        throw std::runtime_error("Robot is not connected.");
    case Status::ERROR:
        throw std::runtime_error("Robot status error.");
    case Status::DISABLED:
    case Status::ENABLED:
    case Status::CONNECTED:
    default:
        break;
    }

    // enable motors by group
    this->enable_motors(this->motors_manipulator);
    this->enable_motors(this->motors_gripper);

    this->status = Status::ENABLED;

    // also checks that every motor now reports enabled
    this->refresh();
}


void UlixArm::disable_motors(const std::vector<Motor>& motors)
{
    for (const auto& motor : motors) {

        try
        {
            // pack frame
            SendFrame send_frame = this->protocol.encode_disable_command(
                motor.get_slave_id()
            );

            // send frame
            this->port.write(send_frame.data, sizeof(send_frame.data));
        }
        catch(...)
        {
            std::throw_with_nested(
                std::runtime_error("Failed to disable motor: " + motor.get_joint_name())
            );
        }
    }
}


void UlixArm::disable()
{
    // check status
    switch (this->status)
    {
    case Status::IDLE:
        throw std::runtime_error("Robot is not connected.");
    case Status::ERROR:
        throw std::runtime_error("Robot status error.");
    case Status::DISABLED:
    case Status::ENABLED:
    case Status::CONNECTED:
    default:
        break;
    }

    // disable motor by group
    this->disable_motors(this->motors_manipulator);
    this->disable_motors(this->motors_gripper);

    this->status = Status::DISABLED;
}


void UlixArm::refresh_motors(std::vector<Motor>& motors)
{
    // pack frames
    std::array<SendFrame, kNumJoints> frames;
    for (std::size_t i = 0; i < motors.size(); ++i) {
        frames[i] = this->protocol.encode_refresh_command(motors[i].get_slave_id());
    }

    try
    {
        // send all frames, then read all responses
        this->send_frames(frames.data(), motors.size());
        this->read_feedbacks(motors);
    }
    catch(...)
    {
        std::throw_with_nested(
            std::runtime_error("Failed to refresh motors.")
        );
    }
}


void UlixArm::refresh()
{
    // check status
    switch (this->status)
    {
    case Status::IDLE:
        throw std::runtime_error("Robot is not connected.");
    case Status::ERROR:
        throw std::runtime_error("Robot status error.");
    case Status::DISABLED:
    case Status::ENABLED:
    case Status::CONNECTED:
    default:
        break;
    }

    // refresh motors by group
    this->refresh_motors(this->motors_manipulator);
    this->refresh_motors(this->motors_gripper);

    // update motor_status_timestamp
    this->motors_update_timestamp = std::chrono::steady_clock::now();
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
    // check status
    this->check_robot_enabled();

    // compute tau gravity
    Joints tau = tau_target;
    if (gravity)
    {
        tau += this->compute_tau_gravity(this->get_q());
    }

    // check joint limits
    try
    {
        this->check_manipulator_joint_limits(q_target, dq_target, tau, kp, kd);
        this->check_position_step(q_target, kp);
        this->check_motor_temperatures();
    }
    catch(...)
    {
        std::throw_with_nested(
            std::runtime_error("Invalid target command.")
        );
    }

    // pack frames
    std::array<SendFrame, kNumJoints> frames;
    for (std::size_t i = 0; i < kNumJoints; ++i) {

        const auto& motor = this->motors_manipulator[i];

        frames[i] = this->protocol.encode_control_command(
            motor.get_slave_id(),
            motor.get_limit_param(),
            q_target[i],
            dq_target[i],
            tau[i],
            kp[i],
            kd[i]
        );
    }

    // send all frames, then read all responses
    try
    {
        this->send_frames(frames.data(), kNumJoints);
        this->read_feedbacks(this->motors_manipulator);
    }
    catch(...)
    {
        std::throw_with_nested(
            std::runtime_error("Failed to send motor commands.")
        );
    }

    // update motors_update_timestamp
    this->motors_update_timestamp = std::chrono::steady_clock::now();
}


void UlixArm::hold()
{
    Joints kp_hold;
    kp_hold << 50.0, 100.0, 60.0, 30.0, 20.0, 10.0;

    Joints kd_hold;
    kd_hold << 1.0, 2.0, 1.5, 0.8, 0.5, 0.2;

    // update motor status to latest position
    this->refresh();

    // command to hold current position
    this->control(this->get_q(), Joints::Zero(), Joints::Zero(), kp_hold, kd_hold, true);
}


void UlixArm::send_frames(const SendFrame* frames, std::size_t n)
{
    if (n == 0)
        return;

    // drop stale frames, then send all frames in one write
    this->port.flush_input();
    this->port.write(frames[0].data, n * sizeof(SendFrame));
}


void UlixArm::read_feedbacks(std::vector<Motor>& motors)
{
    std::array<bool, kNumJoints> received{};
    std::size_t n_received = 0;
    const std::size_t max_reads = motors.size() + this->max_skipped_frames;

    for (std::size_t reads = 0; n_received < motors.size() && reads < max_reads; ++reads)
    {
        // read raw bytes from port
        ReceiveFrame receive_frame;
        try
        {
            this->port.read(
                receive_frame.data,
                this->protocol.head,
                this->protocol.tail,
                sizeof(receive_frame.data)
            );
        }
        catch(...)
        {
            std::throw_with_nested(
                std::runtime_error("Could not read feedback.")
            );
        }

        // parse master id and payload, skip bridge status frames
        MotorID master_id;
        Payload payload;
        if (!this->protocol.parse_usb_bridge_frame(receive_frame, master_id, payload))
            continue;

        // match the frame to its motor
        for (std::size_t i = 0; i < motors.size(); ++i)
        {
            if (!received[i] && motors[i].get_master_id() == master_id)
            {
                this->update_motor_state(motors[i], payload);
                received[i] = true;
                n_received++;
                break;
            }
        }
    }

    for (std::size_t i = 0; i < motors.size(); ++i)
    {
        if (!received[i])
        {
            throw std::runtime_error(
                "No feedback received from motor " + motors[i].get_joint_name()
            );
        }
    }
}


void UlixArm::update_motor_state(Motor& motor, const Payload& payload)
{
    // check the frame really comes from this motor
    if (this->protocol.feedback_slave_id(payload) != (motor.get_slave_id() & 0x0F))
    {
        throw std::runtime_error(
            "Feedback of motor " + motor.get_joint_name() + " carries a different slave id."
        );
    }

    // check motor fault flags, a disabled motor is a fault while the arm is enabled
    try
    {
        this->protocol.check_motor_state(payload, this->status == Status::ENABLED);
    }
    catch(...)
    {
        std::throw_with_nested(
            std::runtime_error("Motor fault on " + motor.get_joint_name())
        );
    }

    // decode payload
    double q, dq, tau;
    int trot, tmos;
    this->protocol.decode_control_feedback(
        payload,
        motor.get_limit_param(),
        q, dq, tau,
        trot, tmos
    );

    // update internal state
    motor.set_q(q);
    motor.set_dq(dq);
    motor.set_tau(tau);
    motor.set_trot(trot);
    motor.set_tmos(tmos);
}


void UlixArm::check_motor_data_age() const
{
    const auto motors_update_age = std::chrono::steady_clock::now() - this->motors_update_timestamp;

    if (motors_update_age > this->max_motors_update_age)
    {
        throw std::runtime_error(
            "Cannot consume motor data older than " +
            std::to_string(this->max_motors_update_age.count()) + " ms."
        );
    }
}

Joints UlixArm::get_q() const
{
    this->check_motor_data_age();

    Joints q;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        q[i] = this->motors_manipulator[i].get_q();
    }
    return q;
}

Joints UlixArm::get_dq() const
{
    this->check_motor_data_age();

    Joints dq;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        dq[i] = this->motors_manipulator[i].get_dq();
    }
    return dq;
}

Joints UlixArm::get_tau() const
{
    this->check_motor_data_age();

    Joints tau;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        tau[i] = this->motors_manipulator[i].get_tau();
    }
    return tau;
}

Joints UlixArm::get_tmos() const
{
    this->check_motor_data_age();

    Joints tmos;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        tmos[i] = static_cast<double>(this->motors_manipulator[i].get_tmos());
    }
    return tmos;
}

Joints UlixArm::get_trot() const
{
    this->check_motor_data_age();

    Joints trot;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        trot[i] = static_cast<double>(this->motors_manipulator[i].get_trot());
    }
    return trot;
}


Joints UlixArm::compute_tau_gravity(const Joints& q)
{
    // write the manipulator joints into the full configuration
    for (std::size_t i = 0; i < kNumJoints; ++i)
    {
        this->pin_q[this->joint_idx_q[i]] = q[i];
    }

    const Eigen::VectorXd& tau_full =
        pinocchio::computeGeneralizedGravity(
            this->pin_model,
            this->pin_data,
            this->pin_q
        );

    Joints tau_gravity;
    for (std::size_t i = 0; i < kNumJoints; ++i)
    {
        tau_gravity[i] = tau_full[this->joint_idx_v[i]];
    }

    return tau_gravity;
}


void UlixArm::check_manipulator_joint_limits(const Joints& q_target, const Joints& dq_target,
    const Joints& tau_target, const Joints& kp, const Joints& kd) const
{
    for (std::size_t i = 0; i < kNumJoints; ++i) {

        const auto& motor = this->motors_manipulator[i];
        const MotorLimitParam& limit_param = motor.get_limit_param();
        const int q_idx = this->joint_idx_q[i];
        const int v_idx = this->joint_idx_v[i];

        // Check joint position limits, negated comparison so that NaN fails the check
        const double q = q_target[i];

        if (!(q >= this->pin_model.lowerPositionLimit[q_idx] &&
              q <= this->pin_model.upperPositionLimit[q_idx]))
        {
            throw std::out_of_range(
                "Joint '" + motor.get_joint_name() + "' position limit exceeded."
            );
        }

        // Effective velocity limit: minimum of URDF and configured limits
        const double max_dq = std::min(this->pin_model.velocityLimit[v_idx], limit_param.dq);

        if (!(std::abs(dq_target[i]) <= max_dq))
        {
            throw std::out_of_range(
                "Joint '" + motor.get_joint_name() + "' velocity limit exceeded."
            );
        }

        // Effective effort limit: minimum of URDF and configured limits
        const double max_tau = std::min(this->pin_model.effortLimit[v_idx], limit_param.tau);

        if (!(std::abs(tau_target[i]) <= max_tau))
        {
            throw std::out_of_range(
                "Joint '" + motor.get_joint_name() + "' effort limit exceeded."
            );
        }

        // Gains: range supported by the motor protocol
        if (!(kp[i] >= 0.0 && kp[i] <= DamiaoProtocol::kp_max))
        {
            throw std::out_of_range(
                "Joint '" + motor.get_joint_name() + "' kp out of range [0, 500]."
            );
        }

        if (!(kd[i] >= 0.0 && kd[i] <= DamiaoProtocol::kd_max))
        {
            throw std::out_of_range(
                "Joint '" + motor.get_joint_name() + "' kd out of range [0, 5]."
            );
        }
    }
}


void UlixArm::check_position_step(const Joints& q_target, const Joints& kp) const
{
    // no stiffness -> target position has no effect
    if ((kp.array() <= 0.0).all())
        return;

    const Joints q = this->get_q();

    for (std::size_t i = 0; i < kNumJoints; ++i) {

        if (kp[i] > 0.0 && std::abs(q_target[i] - q[i]) > this->max_q_step)
        {
            throw std::out_of_range(
                "Joint '" + this->motors_manipulator[i].get_joint_name() +
                "' target is " + std::to_string(std::abs(q_target[i] - q[i])) +
                " rad away from current position (max " + std::to_string(this->max_q_step) +
                " rad). Interpolate the motion."
            );
        }
    }
}


void UlixArm::check_motor_temperatures()
{
    bool any_motor_warm = false;

    for (const auto* motors : {&this->motors_manipulator, &this->motors_gripper}) {
        for (const auto& motor : *motors) {

            const int temperature = std::max(motor.get_tmos(), motor.get_trot());

            if (temperature > this->max_motor_temperature)
            {
                throw std::runtime_error(
                    "Motor '" + motor.get_joint_name() + "' overheating: MOS " +
                    std::to_string(motor.get_tmos()) + " C, rotor " +
                    std::to_string(motor.get_trot()) + " C (max " +
                    std::to_string(this->max_motor_temperature) + " C)."
                );
            }

            if (temperature > this->warn_motor_temperature)
            {
                any_motor_warm = true;

                // print only once, to not flood the control loop
                if (!this->temperature_warning_printed)
                {
                    std::cerr << "Warning: motor '" << motor.get_joint_name() << "' is hot: MOS "
                              << motor.get_tmos() << " C, rotor " << motor.get_trot()
                              << " C (error at " << this->max_motor_temperature << " C).\n";
                    this->temperature_warning_printed = true;
                }
            }
        }
    }

    // all motors cooled down -> warn again next time
    if (!any_motor_warm)
    {
        this->temperature_warning_printed = false;
    }
}


void UlixArm::check_robot_enabled() const
{
    switch (this->status)
    {
    case Status::IDLE:
        throw std::runtime_error("Robot is not connected.");
    case Status::DISABLED:
    case Status::CONNECTED:
        throw std::runtime_error("Robot is not enabled.");
    case Status::ERROR:
        throw std::runtime_error("Robot status error.");
    case Status::ENABLED:
    default:
        break;
    }
}


void UlixArm::check_gripper_mounted() const
{
    if (!this->gripper || this->motors_gripper.empty())
    {
        throw std::runtime_error("No gripper mounted.");
    }
}


void UlixArm::log_errors(const std::exception& e, int level)
{
    std::cerr << std::string(level * 2, ' ')
              << (level == 0 ? "Error: " : "Caused by: ")
              << e.what() << '\n';

    try {
        std::rethrow_if_nested(e);
    }
    catch (const std::exception& nested) {
        this->log_errors(nested, level + 1);
    }
    catch (...) {
        std::cerr << std::string((level + 1) * 2, ' ')
                  << "Caused by: Unknown exception\n";
    }
}


double UlixArm::gripper_ik(double d) const
{
    constexpr double l1 = 0.033;    // rotor link
    constexpr double l2 = 0.035;    // rod link
    constexpr double e  = -0.0156;  // offset perpendicular to rail axis
    constexpr double f  = 0.0165;   // offset parallel to rail axis

    const double A = d + f;

    const double R = std::hypot(A, e);

    const double K =
        (A * A + e * e + l1 * l1 - l2 * l2)
        / (2.0 * l1);

    // Protect acos() against floating-point roundoff.
    double Q = std::clamp(K / R, -1.0, 1.0);

    // Select the solution branch satisfying -1.5 <= th1 <= 0.
    const double th1 = std::atan2(e, A) - std::acos(Q);

    return th1;
}


void UlixArm::open()
{
    this->open_ratio(0.8);
}


void UlixArm::open_ratio(double ratio)
{
    this->open_width(ratio * this->max_gripper_opening);
}


void UlixArm::open_width(double meters)
{
    this->check_gripper_mounted();
    this->check_robot_enabled();

    // finger joint moves half the opening
    double d_half = meters / 2.0;

    // throw if joint limits exceed
    this->check_gripper_joint_limits(d_half);

    // compute angle of motor -> motor angle is zero for opening zero.
    double q_open = this->gripper_ik(0.0) - this->gripper_ik(d_half);

    this->control_gripper(q_open, 0.0, 4.0, 0.5);
}

void UlixArm::close()
{
    this->close(0.5);   // default: close with half force
}

void UlixArm::close(double f)
{
    this->check_gripper_mounted();
    this->check_robot_enabled();

    // negated comparison so that NaN fails the check
    if (!(f >= 0.0 && f <= 1.0))
    {
        throw std::out_of_range(
            "Force factor limit exceeded. Accepted range [0,1], got " + std::to_string(f)
        );
    }

    double tau_close_min = 0.2;     // min torque of motor to close gripper in Nm (from heuristic)
    double tau_close_max = 1.2;     // max torque of motor to not break fingers in Nm (from heuristic)
    double tau_close = tau_close_min + f * (tau_close_max - tau_close_min);

    this->control_gripper(0.0, tau_close, 0.0, 0.1);
}


void UlixArm::control_gripper(double q, double tau, double kp, double kd)
{
    auto& motor = this->motors_gripper[0];

    try
    {
        // pack frame
        SendFrame send_frame = this->protocol.encode_control_command(
            motor.get_slave_id(),
            motor.get_limit_param(),
            q,
            0.0,    // dq
            tau,
            kp,
            kd
        );

        // send frame, then read response
        this->send_frames(&send_frame, 1);
        this->read_feedbacks(this->motors_gripper);
    }
    catch(...)
    {
        std::throw_with_nested(
            std::runtime_error("Failed to control motor " + motor.get_joint_name())
        );
    }
}


void UlixArm::check_gripper_joint_limits(double d_target) const
{
    // negated comparison so that NaN fails the check
    if (!(d_target >= this->pin_model.lowerPositionLimit[this->gripper_idx_q] &&
          d_target <= this->pin_model.upperPositionLimit[this->gripper_idx_q]))
    {
        throw std::out_of_range(
            "Joint '" + this->motors_gripper[0].get_joint_name() + "' position limit exceeded."
        );
    }
}

} // namespace ulixarm
