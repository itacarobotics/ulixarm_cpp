#include "ulixarm.hpp"

#include <algorithm>


UlixArm::UlixArm(bool gripper)
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
    const std::string package_path = ULIXARM_DESCRIPTION_PATH;
    const std::string urdf_path = package_path + "/urdf/robot.urdf";

    try
    {
        pinocchio::urdf::buildModel(urdf_path, this->pin_model);
        this->pin_data = pinocchio::Data(this->pin_model);
    }
    catch(...)
    {
        std::throw_with_nested(
            std::invalid_argument("Urdf file not found. Make sure to keep structure of folder.")
        );
    }

    // init status
    this->status = UlixArmStatus::IDLE;
    return;
}

UlixArm::~UlixArm()
{
    // serial port is automatically closed from desctructor.

    // keep motors enabled to avoid arm falling.

    return;
}


void UlixArm::connect(const char* port)
{
    // check status
    switch (this->status)
    {
    case UlixArmStatus::CONNECTED:
    case UlixArmStatus::DISABLED:
    case UlixArmStatus::ENABLED:
        throw std::runtime_error("Robot already connected.");
    case UlixArmStatus::ERROR:
        throw std::runtime_error("Robot status error.");
    case UlixArmStatus::IDLE:
    default:
        break;
    }

    // open bus
    try
    {
        this->port.init(
            port,       // name of port
            static_cast <unsigned int>(921600), // baudrate [byte/s]
            static_cast <unsigned int>(10)      // byte timeout [ms]
        );
    }
    catch(...)
    {
        std::throw_with_nested(
            std::runtime_error("Failed to init port.")
        );
    }

    this->status = UlixArmStatus::CONNECTED;
    return;
}

void UlixArm::enable_motors(const std::vector<Motor>& motors)
{
    for (const auto& motor : motors) {
        
        try
        {
            // pack frame
            SendFrame send_frame;
            send_frame = this->protocol.encode_enable_command(
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
    case UlixArmStatus::IDLE:
        throw std::runtime_error("Robot is not connected.");
    case UlixArmStatus::ERROR:
        throw std::runtime_error("Robot status error.");
    case UlixArmStatus::DISABLED:
    case UlixArmStatus::ENABLED:
    case UlixArmStatus::CONNECTED:
    default:
        break;
    }

    // enable motors by group
    this->enable_motors(this->motors_manipulator);
    this->enable_motors(this->motors_gripper);

    this->status = UlixArmStatus::ENABLED;

    this->refresh();
    return;
}


void UlixArm::disable_motors(const std::vector<Motor>& motors)
{
    for (const auto& motor : motors) {
        
        try
        {
            // pack frame
            SendFrame send_frame;
            send_frame = this->protocol.encode_disable_command(
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


void UlixArm::disable()
{
    // check status
    switch (this->status)
    {
    case UlixArmStatus::IDLE:
        throw std::runtime_error("Robot is not connected.");
    case UlixArmStatus::ERROR:
        throw std::runtime_error("Robot status error.");
    case UlixArmStatus::DISABLED:
    case UlixArmStatus::ENABLED:
    case UlixArmStatus::CONNECTED:
    default:
        break;
    }

    // disable motor by group
    this->disable_motors(this->motors_manipulator);
    this->disable_motors(this->motors_gripper);

    this->status = UlixArmStatus::DISABLED;
    return;
}


void UlixArm::refresh_motors(std::vector<Motor>& motors)
{
    for (auto& motor : motors) {

        try
        {
            // pack frame
            SendFrame send_frame;
            send_frame = this->protocol.encode_refresh_command(
                motor.get_slave_id()
            );
    
            // drop stale frames, then send frame
            this->port.flush_input();
            this->port.write(send_frame.data, sizeof(send_frame.data));
            
            // read response
            this->read_feedback(motor);
        }
        catch(...)
        {
            std::throw_with_nested(
                std::runtime_error("Failed to refresh motor " + motor.get_joint_name())
            );
            return;
        }
        
    }
}


void UlixArm::refresh()
{
    // check status
    this->check_robot_enabled();

    // refresh motors by group
    this->refresh_motors(this->motors_manipulator);
    this->refresh_motors(this->motors_gripper);

    // update motor_status_timestamp
    this->motors_update_timestamp = std::chrono::steady_clock::now();

    return;
}


void UlixArm::control(
    Joints q_target,
    Joints dq_target,
    Joints tau_target,
    Joints kp,
    Joints kd,
    bool gravity
)
{
    // check status
    this->check_robot_enabled();

    // compute tau gravity
    if (gravity)
    {
        Joints tau_gravity = this->compute_tau_gravity(this->get_q());
        tau_target += tau_gravity;
    }

    // check joint limits
    try
    {
        this->check_manipulator_joint_limits(q_target, dq_target, tau_target, kp, kd);
        this->check_position_step(q_target, kp);
        this->check_motor_temperatures();
    }
    catch(...)
    {
        std::throw_with_nested(
            std::runtime_error("Invalid target command.")
        );
    }

    // send commands
    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        
        auto& motor = this->motors_manipulator[i];

        try
        {
            // pack frame
            SendFrame send_frame;
            send_frame = this->protocol.encode_control_command(
                motor.get_slave_id(),
                motor.get_limit_param(),
                q_target[i],
                dq_target[i],
                tau_target[i],
                kp[i],
                kd[i]
            );
    
            // drop stale frames, then send frame
            this->port.flush_input();
            this->port.write(send_frame.data, sizeof(send_frame.data));
    
            // read response
            this->read_feedback(motor);
        }
        catch(...)
        {
            std::throw_with_nested(
                std::runtime_error("Failed to send motor commands to " + motor.get_joint_name())
            );
        }

    }

    // update motors_update_timestamp
    this->motors_update_timestamp = std::chrono::steady_clock::now();

    return;
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
    
    return;
}


void UlixArm::read_feedback(Motor& motor)
{
    // bridge status frames (heartbeat, send ack) and frames from other motors are skipped
    constexpr int max_skipped_frames = 8;

    MotorID master_id;
    Payload payload;
    bool found = false;

    for (int i = 0; i <= max_skipped_frames; ++i)
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
        
        // parse master id and payload
        const bool has_can_data = this->protocol.parse_usb_bridge_frame(
            receive_frame,
            master_id,
            payload
        );

        found = has_can_data && (master_id == motor.get_master_id());
        if (found)
            break;

    }
    
    if (!found)
    {
        throw std::runtime_error(
            "No feedback received from motor " + motor.get_joint_name()
        );
    }

    // check motor fault flags
    try
    {
        this->protocol.check_motor_state(payload);
    }
    catch(...)
    {
        std::throw_with_nested(
            std::runtime_error("Motor fault on " + motor.get_joint_name())
        );
    }

    // decode payload
    MotorLimitParam limit_param = motor.get_limit_param();

    double q, dq, tau;
    int trot, tmos;
    this->protocol.decode_control_feedback(
        payload,
        limit_param,
        q, dq, tau,
        trot, tmos            
    );

    // update internal state
    motor.set_q(q);
    motor.set_dq(dq);
    motor.set_tau(tau);
    motor.set_trot(trot);
    motor.set_tmos(tmos);

    return;
}

Joints UlixArm::get_q()
{
    const auto motors_update_age = std::chrono::steady_clock::now() - this->motors_update_timestamp;

    if (motors_update_age > this->max_motors_update_age)
    {
        throw std::runtime_error(
                "Cannot consume motor data older than " +
                std::to_string(max_motors_update_age.count()) + " ms."
            );
    }

    Joints q = Joints::Zero();
    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        q[i] = this->motors_manipulator[i].get_q();
    }
    return q;
}

Joints UlixArm::get_dq()
{
    const auto motors_update_age = std::chrono::steady_clock::now() - this->motors_update_timestamp;

    if (motors_update_age > this->max_motors_update_age)
    {
        throw std::runtime_error(
                "Cannot consume motor data older than " +
                std::to_string(max_motors_update_age.count()) + " ms."
            );
    }

    Joints dq = Joints::Zero();
    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        dq[i] = this->motors_manipulator[i].get_dq();
    }
    return dq;
}

Joints UlixArm::get_tau()
{
    const auto motors_update_age = std::chrono::steady_clock::now() - this->motors_update_timestamp;

    if (motors_update_age > this->max_motors_update_age)
    {
        throw std::runtime_error(
                "Cannot consume motor data older than " +
                std::to_string(max_motors_update_age.count()) + " ms."
            );
    }

    Joints tau = Joints::Zero();
    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        tau[i] = this->motors_manipulator[i].get_tau();
    }
    return tau;
}

Joints UlixArm::get_tmos()
{
    const auto motors_update_age = std::chrono::steady_clock::now() - this->motors_update_timestamp;

    if (motors_update_age > this->max_motors_update_age)
    {
        throw std::runtime_error(
                "Cannot consume motor data older than " +
                std::to_string(max_motors_update_age.count()) + " ms."
            );
    }

    Joints tmos = Joints::Zero();
    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        tmos[i] = static_cast<double>(this->motors_manipulator[i].get_tmos());
    }
    return tmos;
}

Joints UlixArm::get_trot()
{
    const auto motors_update_age = std::chrono::steady_clock::now() - this->motors_update_timestamp;

    if (motors_update_age > this->max_motors_update_age)
    {
        throw std::runtime_error(
                "Cannot consume motor data older than " +
                std::to_string(max_motors_update_age.count()) + " ms."
            );
    }

    Joints trot = Joints::Zero();
    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        trot[i] = static_cast<double>(this->motors_manipulator[i].get_trot());
    }
    return trot;
}

Joints UlixArm::compute_tau_gravity(Joints q)
{
    Joints tau_gravity = Joints::Zero();

    // urdf might have >7 joints (two fingers and gripper)
    Eigen::VectorXd q_full = pinocchio::neutral(this->pin_model);

    for (size_t i = 0; i < this->motors_manipulator.size(); i++)
    {
        q_full[i] = q[i];
    }

    Eigen::VectorXd tau_full =
        pinocchio::computeGeneralizedGravity(
            this->pin_model,
            this->pin_data,
            q_full
        );

    for (size_t i = 0; i < this->motors_manipulator.size(); i++)
    {
        tau_gravity[i] = tau_full[i];
    }

    return tau_gravity;
}


void UlixArm::check_manipulator_joint_limits(Joints q_target, Joints dq_target, Joints tau_target, Joints kp, Joints kd)
{
    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        
        const auto& motor = this->motors_manipulator[i];
        const std::string joint_name = motor.get_joint_name();
        const MotorLimitParam limit_param = motor.get_limit_param();

        // Get joint ID
        const pinocchio::JointIndex joint_id =
            this->pin_model.getJointId(joint_name);

        if (joint_id == 0 || joint_id >= this->pin_model.njoints) {
            throw std::out_of_range(
                "Joint '" + joint_name + "' not found in Pinocchio model."
            );
        }

        const auto &joint = this->pin_model.joints[joint_id];

        // Check joint position limits
        const auto q_idx = joint.idx_q();
        const double q = q_target[i];

        if (!std::isfinite(q)) 
        {
            throw std::runtime_error(
                "Joint '" + joint_name + "' has a non-finite position."
            );
        }

        if (q < this->pin_model.lowerPositionLimit[q_idx] ||
            q > this->pin_model.upperPositionLimit[q_idx]) 
        {
            throw std::out_of_range(
                "Joint '" + joint_name + "' position limit exceeded."
            );
        }

        // Effective velocity limit: minimum of URDF and configured limits
        const double urdf_dq = this->pin_model.velocityLimit[joint.idx_v()];
        const double max_dq = std::min(urdf_dq, limit_param.dq);
        const double dq = dq_target[i];

        // negated comparison so that NaN fails the check
        if (!(std::abs(dq) <= max_dq)) 
        {
            throw std::out_of_range(
                "Joint '" + joint_name + "' velocity limit exceeded."
            );
        }

        // Effective effort limit: minimum of URDF and configured limits
        const double urdf_tau = this->pin_model.effortLimit[joint.idx_v()];
        const double max_tau = std::min(urdf_tau, limit_param.tau);
        const double tau = tau_target[i];

        if (!(std::abs(tau) <= max_tau)) 
        {
            throw std::out_of_range(
                "Joint '" + joint_name + "' effort limit exceeded."
            );
        }

        // Gains: range supported by the motor protocol
        if (!(kp[i] >= 0.0 && kp[i] <= 500.0)) 
        {
            throw std::out_of_range(
                "Joint '" + joint_name + "' kp out of range [0, 500]."
            );
        }

        if (!(kd[i] >= 0.0 && kd[i] <= 5.0)) 
        {
            throw std::out_of_range(
                "Joint '" + joint_name + "' kd out of range [0, 5]."
            );
        }
    }
}


void UlixArm::check_position_step(Joints q_target, Joints kp)
{
    // no stiffness -> target position has no effect
    if ((kp.array() <= 0.0).all())
        return;

    const Joints q = this->get_q();

    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        
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
    for (const auto* motors : {&this->motors_manipulator, &this->motors_gripper}) {
        for (const auto& motor : *motors) {
            
            if (motor.get_tmos() > this->max_motor_temperature ||
                motor.get_trot() > this->max_motor_temperature)
            {
                throw std::runtime_error(
                    "Motor '" + motor.get_joint_name() + "' overheating: MOS " +
                    std::to_string(motor.get_tmos()) + " C, rotor " +
                    std::to_string(motor.get_trot()) + " C (max " +
                    std::to_string(this->max_motor_temperature) + " C)."
                );
            }
        }
    }
}


void UlixArm::check_robot_enabled()
{
    switch (this->status)
    {
    case UlixArmStatus::IDLE:
        throw std::runtime_error("Robot is not connected.");
    case UlixArmStatus::DISABLED:
    case UlixArmStatus::CONNECTED:
        throw std::runtime_error("Robot is not enabled.");
    case UlixArmStatus::ERROR:
        throw std::runtime_error("Robot status error.");
    case UlixArmStatus::ENABLED:
    default:
        break;
    }
}


void UlixArm::check_gripper_mounted()
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


double UlixArm::gripper_ik(double d)
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
    this->open(0.8, "normal");
}


void UlixArm::open(double d, std::string type)
{
    this->check_gripper_mounted();
    this->check_robot_enabled();

    double d_half;
    if (type == "metric")
    {
        // finger joint moves half the opening
        d_half = d / 2.0;
    }
    else if (type == "normal")
    {
        d_half = (d * this->max_gripper_opening) / 2.0;
    }
    else
    {
        throw std::invalid_argument("Invalid argument. Should be one of [\"metric\", \"normal\"]");
    }

    // throw if joint limits exceed
    this->check_gripper_joint_limits(d_half);
    
    // compute angle of motor -> motor angle is zero for opening zero.
    double q_open = this->gripper_ik(0.0) - this->gripper_ik(d_half);
    
    auto& motor = this->motors_gripper[0];
    
    try
    {
        // pack frame
        SendFrame send_frame;
        send_frame = this->protocol.encode_control_command(
            motor.get_slave_id(),
            motor.get_limit_param(),
            q_open,     // q
            0.0,        // dq
            0.0,        // tau
            4.0,        // kp
            0.5         // kd
        );

        // drop stale frames, then send frame
        this->port.flush_input();
        this->port.write(send_frame.data, sizeof(send_frame.data));

        // read response
        this->read_feedback(motor);
    }
    catch(...)
    {
        std::throw_with_nested(
            std::runtime_error("Failed to control motor " + motor.get_joint_name())
        );
        return;
    }
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
    double tau_close_max = 1.2;     // min torque of motor to not break fingers in Nm (from heuristic)
    double tau_close = tau_close_min + f * (tau_close_max - tau_close_min);

    auto& motor = this->motors_gripper[0];

    try
    {
        // pack frame
        SendFrame send_frame;
        send_frame = this->protocol.encode_control_command(
            motor.get_slave_id(),
            motor.get_limit_param(),
            0.0,
            0.0,
            tau_close,
            0.0,
            0.1
        );

        // drop stale frames, then send frame
        this->port.flush_input();
        this->port.write(send_frame.data, sizeof(send_frame.data));

        // read response
        this->read_feedback(motor);
    }
    catch(...)
    {
        std::throw_with_nested(
            std::runtime_error("Failed to control motor " + motor.get_joint_name())
        );
        return;
    }
}


void UlixArm::check_gripper_joint_limits(double d_target)
{
    this->check_gripper_mounted();

    const auto& motor = this->motors_gripper[0];
    const std::string joint_name = motor.get_joint_name();

    // Get joint ID
    const pinocchio::JointIndex joint_id =
        this->pin_model.getJointId(joint_name);

    if (joint_id == 0 || joint_id >= this->pin_model.njoints) 
    {
        throw std::out_of_range(
            "Joint '" + joint_name + "' not found in Pinocchio model."
        );
    }

    const auto &joint = this->pin_model.joints[joint_id];

    // Check joint position limits
    const auto q_idx = joint.idx_q();
    const double d = d_target;

    if (!std::isfinite(d)) 
    {
        throw std::runtime_error(
            "Joint '" + joint_name + "' has a non-finite position."
        );
    }

    if (d < this->pin_model.lowerPositionLimit[q_idx] ||
        d > this->pin_model.upperPositionLimit[q_idx]) 
    {
        throw std::out_of_range(
            "Joint '" + joint_name + "' position limit exceeded."
        );
    }

    return;
}
