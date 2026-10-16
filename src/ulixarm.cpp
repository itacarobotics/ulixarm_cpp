#include "ulixarm_cpp/ulixarm.hpp"

#include <iomanip>


UlixArm::UlixArm(bool gripper)
{
    this->gripper = gripper;

    this->status = UlixArmStatus::IDLE;
    
    // add motors
    this->motors_manipulator.emplace_back(0x01, 0x11, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}, "joint1"); // DM4340p
    this->motors_manipulator.emplace_back(0x02, 0x12, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}, "joint2"); // DM4340
    this->motors_manipulator.emplace_back(0x03, 0x13, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}, "joint3"); // DM4340
    this->motors_manipulator.emplace_back(0x04, 0x14, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}, "joint4"); // DM4310
    this->motors_manipulator.emplace_back(0x05, 0x15, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}, "joint5"); // DM4310
    this->motors_manipulator.emplace_back(0x06, 0x16, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}, "joint6"); // DM4310
    
    if (this->gripper)
    {
        this->motors_gripper.emplace_back(0x07, 0x17, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}, "gripper"); // DM4310
    }
    
    // create pinocchio model and data
    const std::string urdf_path = "ulixarm_description/src/ulixarm_description/urdf/robot.urdf";
    pinocchio::urdf::buildModel(urdf_path, this->pin_model);
    this->pin_data = pinocchio::Data(this->pin_model);

    return;
}

UlixArm::~UlixArm()
{
    // close serial
    // this->disable();
    return;
}


void UlixArm::connect(const char* port)
{
    // check status

    // open bus
    try
    {
        this->port.init(port, this->baudrate, this->timeout_ms);
    }
    catch(...)
    {
        std::throw_with_nested(
            std::runtime_error("Failed to init port.")
        );
        return;
    }

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
    // check status if connected
    
    this->enable_motors(this->motors_manipulator);
    this->enable_motors(this->motors_gripper);

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
    // check status if connected
    
    this->disable_motors(this->motors_manipulator);
    this->disable_motors(this->motors_gripper);

    return;
}


void UlixArm::refresh_motors(const std::vector<Motor>& motors)
{
    for (const auto& motor : motors) {
        
        try
        {
            // pack frame
            SendFrame send_frame;
            send_frame = this->protocol.encode_refresh_command(
                motor.get_slave_id()
            );
    
            // send frame
            this->port.write(send_frame.data, sizeof(send_frame.data));
            
            // read response
            this->read_feedback();
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
    this->refresh_motors(this->motors_manipulator);
    this->refresh_motors(this->motors_gripper);

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

    // compute tau gravity
    if (gravity)
    {
        Joints tau_gravity = this->compute_tau_gravity(this->get_q());
        tau_target += tau_gravity;
    }

    // check collisions
    try
    {
        this->check_joint_limits(q_target, dq_target, tau_target);
    }
    catch(...)
    {
        std::throw_with_nested(
            std::runtime_error("Joint limits detected")
        );
        return;
    }

    // send commands
    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        
        const auto& motor = this->motors_manipulator[i];

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
    
            // send frame
            this->port.write(send_frame.data, sizeof(send_frame.data));
    
            // read response
            this->read_feedback();
        }
        catch(...)
        {
            std::throw_with_nested(
                std::runtime_error("Failed to control motor " + motor.get_joint_name())
            );
            return;
        }

    }

    return;
}


void UlixArm::hold()
{
    this->refresh();

    Joints kp, kd;
    kp << 50.0, 100.0, 60.0, 30.0, 20.0, 10.0, 0.0;
    kd << 1.0, 2.0, 1.5, 0.8, 0.5, 0.2, 1.0;
    this->control(this->get_q(), Joints::Zero(), Joints::Zero(), kp, kd, true);
    
    return;
}


void UlixArm::read_feedback()
{
    // read raw bytes from port
    ReceiveFrame receive_frame;
    this->port.read(
        receive_frame.data, 
        this->protocol.head,
        this->protocol.tail,
        sizeof(receive_frame.data)
    );

    // parse master id and payload
    MotorID master_id;
    Payload payload;
    this->protocol.parse_usb_bridge_frame(
        receive_frame,
        master_id,
        payload
    );

    // decode payload
    Motor* motor = this->find_motor_by_master_id(master_id);
    MotorLimitParam limit_param = motor->get_limit_param();

    double q, dq, tau;
    int trot, tmos;
    this->protocol.decode_control_feedback(
        payload,
        limit_param,
        q, dq, tau,
        trot, tmos            
    );

    // update internal state
    motor->set_q(q);
    motor->set_dq(dq);
    motor->set_tau(tau);
    motor->set_trot(trot);
    motor->set_tmos(tmos);
}

Joints UlixArm::get_q()
{
    Joints q = Joints::Zero();
    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        q[i] = this->motors_manipulator[i].get_q();
    }
    return q;
}

Joints UlixArm::get_dq()
{
    Joints dq = Joints::Zero();
    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        dq[i] = this->motors_manipulator[i].get_dq();
    }
    return dq;
}

Joints UlixArm::get_tau()
{
    Joints tau = Joints::Zero();
    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        tau[i] = this->motors_manipulator[i].get_tau();
    }
    return tau;
}

Joints UlixArm::get_tmos()
{
    Joints tmos = Joints::Zero();
    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        tmos[i] = static_cast<double>(this->motors_manipulator[i].get_tmos());
    }
    return tmos;
}

Joints UlixArm::get_trot()
{
    Joints trot = Joints::Zero();
    for (std::size_t i = 0; i < this->motors_manipulator.size(); ++i) {
        trot[i] = static_cast<double>(this->motors_manipulator[i].get_trot());
    }
    return trot;
}


Motor* UlixArm::find_motor_by_master_id(MotorID master_id)
{
    for (auto& motor : this->motors_manipulator) {
        if (motor.get_master_id() == master_id) {
            return &motor;
        }
    }

    for (auto& motor : this->motors_gripper) {
        if (motor.get_master_id() == master_id) {
            return &motor;
        }
    }

    return nullptr;
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


double UlixArm::gripper_ik(double s)
{
    constexpr double l1 = 33.0;
    constexpr double l2 = 35.0;
    constexpr double c  = 15.6;
    
    // Calculate cos(theta2).
    double cos_theta2 =
        (s * s + c * c - l1 * l1 - l2 * l2)
        / (2.0 * l1 * l2);
    
    // Clamp to avoid floating-point errors in acos().
    cos_theta2 = std::clamp(cos_theta2, -1.0, 1.0);

    // Select the positive-theta2 branch.
    const double theta2 = std::acos(cos_theta2);
    
    // Calculate theta1.
    const double theta1 =
        std::atan2(c, s)
        - std::atan2(
            l2 * std::sin(theta2),
            l1 + l2 * std::cos(theta2)
        );
    
    return theta1;
}

double UlixArm::gripper_fk(double theta)
{
    constexpr double l1 = 33.0;
    constexpr double l2 = 35.0;
    constexpr double c  = 15.6;

    // Calculate sin(theta1 + theta2).
    double sin_sum = (c - l1 * std::sin(theta)) / l2;

    // Check reachability.
    if (sin_sum < -1.0 || sin_sum > 1.0)
        return -1.0;

    // Select the appropriate branch.
    const double sum_angle = M_PI - std::asin(sin_sum);

    // Calculate s.
    const double s =
        l1 * std::cos(theta)
        + l2 * std::cos(sum_angle);

    return s;
}


void UlixArm::check_joint_limits(Joints q_target, Joints dq_target, Joints tau_target)
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

        if (!std::isfinite(q)) {
            throw std::runtime_error(
                "Joint '" + joint_name + "' has a non-finite position."
            );
        }

        if (q < this->pin_model.lowerPositionLimit[q_idx] ||
            q > this->pin_model.upperPositionLimit[q_idx]) {
            throw std::out_of_range(
                "Joint '" + joint_name + "' position limit exceeded."
            );
        }

        // Effective velocity limit: minimum of URDF and configured limits
        const double urdf_dq = this->pin_model.velocityLimit[joint.idx_v()];
        const double max_dq = std::min(urdf_dq, limit_param.dq);
        const double dq = dq_target[i];

        if (std::abs(dq) > max_dq) {
            throw std::out_of_range(
                "Joint '" + joint_name + "' velocity limit exceeded."
            );
        }

        // Effective effort limit: minimum of URDF and configured limits
        const double urdf_tau = this->pin_model.effortLimit[joint.idx_v()];
        const double max_tau = std::min(urdf_tau, limit_param.tau);
        const double tau = tau_target[i];

        if (std::abs(tau) > max_tau) {
            throw std::out_of_range(
                "Joint '" + joint_name + "' effort limit exceeded."
            );
        }
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