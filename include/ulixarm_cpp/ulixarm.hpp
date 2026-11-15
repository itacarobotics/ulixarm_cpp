#pragma once

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <string>
#include <vector>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include "motor.hpp"
#include "protocol.hpp"
#include "serial_port.hpp"

namespace ulixarm
{

constexpr std::size_t kNumJoints = 6;

// Margin [rad] added on both sides of the URDF position limits of the manipulator joints.
// A URDF range [lo, hi] is accepted as [lo - margin, hi + margin].
constexpr double kJointPositionLimitMargin = 0.1;

using Joints = Eigen::Matrix<double, kNumJoints, 1>;

enum class Status
{
    IDLE = 0,
    CONNECTED,
    ENABLED,
    DISABLED,
    RUNNING,
    ERROR
};

class UlixArm
{
public:

    /**
     * @brief Creates the arm object and sets up everything it needs to run.
     *
     * Loads the kinematic/dynamic model used for gravity compensation and
     * prepares the motor lists for the manipulator joints and, if present,
     * the gripper. It doesn't talk to the hardware yet; call connect()
     * for that.
     *
     * The URDF is looked up in this order:
     * 1. urdf_path, if not empty
     * 2. $ULIXARM_DESCRIPTION_PATH/urdf/robot.urdf, if the environment variable is set
     * 3. the ulixarm_description installed with the library
     * 4. the ulixarm_description submodule of the source tree
     *
     * @param gripper Set to true if a gripper is mounted on the arm, false if not.
     * @param urdf_path Path to the robot URDF. Leave empty to use the lookup above.
     */
    explicit UlixArm(bool gripper = true, const std::string& urdf_path = "");

    /**
     * @brief Cleans up when the arm object goes away.
     *
     * Closes the serial connection. The motors are left enabled so that the
     * arm does not fall; they keep executing the last command.
     */
    ~UlixArm();

    /**
     * @brief Opens the serial connection to the arm.
     *
     * Connects to the motor controller on the given port and checks that every
     * motor runs in MIT mode with the expected PMAX/VMAX/TMAX mapping. This has
     * to happen before enabling the motors or sending any commands.
     *
     * @param port Path of the serial device, for example "/dev/ttyACM0".
     */
    void connect(const char* port);

    /**
     * @brief Closes the serial connection to the arm.
     *
     * Does not disable the motors: they keep executing the last command.
     * Safe to call when not connected.
     */
    void disconnect();

    /**
     * @brief Powers up the motors so the arm can move.
     *
     * Enables the manipulator motors (and the gripper motors if a gripper
     * is mounted). The arm needs to be connected first.
     */
    void enable();

    /**
     * @brief Powers down the motors.
     *
     * Disables the manipulator motors (and the gripper motors if present).
     * Once disabled, the arm will no longer hold its position, so make sure
     * it's in a safe pose first.
     *
     * @attention Disabling the arm will make it fall down, risking to injure people
     * or damage the robot or objects underneath.
     */
    void disable();

    /**
     * @brief Updates the arm's state from the motors.
     *
     * Asks the motors for their latest data and stores it, so get_q(),
     * get_dq(), get_tau() and the temperature getters return current values.
     * Works while connected, enabled or disabled.
     *
     * @note Use this function at initialization only, as the control() function
     * already updates the latest data from the motor feedback.
     */
    void refresh();

    /**
     * @brief Sends a control command to all joints of the manipulator.
     *
     * Each joint follows an impedance control law: a spring pulling it toward
     * the target position (kp), a damper tracking the target velocity (kd),
     * plus a feedforward torque on top. If gravity is true, the torque
     * needed to hold the arm up against gravity is added as well, so the
     * gains don't have to fight the arm's own weight. Before anything is sent,
     * the targets and gains are checked against the joint limits, joints with
     * kp > 0 must be within max_q_step of their current position, and no motor
     * may exceed max_motor_temperature.
     *
     * The success path does not allocate memory, so it can be called from a
     * realtime loop. Errors are reported with exceptions.
     *
     * @note This method does not control the gripper, use open_width(),
     * open_ratio() and close() instead.
     *
     * @warning Self collision is NOT checked.
     *
     * @param q_target Desired joint positions, in rad.
     * @param dq_target Desired joint velocities, in rad/s.
     * @param tau_target Feedforward torques, in Nm.
     * @param kp Position (stiffness) gains, one per joint.
     * @param kd Velocity (damping) gains, one per joint.
     * @param gravity True to add gravity compensation, false to leave it out.
     */
    void control(
        const Joints& q_target,
        const Joints& dq_target,
        const Joints& tau_target,
        const Joints& kp,
        const Joints& kd,
        bool gravity
    );

    /**
     * @brief Keeps the arm where it currently is.
     *
     * Commands the joints to stay at their present positions so the arm
     * stays put instead of drifting or dropping.
     *
     * @note this method calls refresh() internally to get latest motor positions.
     */
    void hold();

    /**
     * @brief Gets the current joint positions.
     *
     * Returns the latest positions reported by the motors. These come from
     * the last feedback received. This can originate from calling refresh()
     * or control().
     *
     * @return The joint angles, in radians.
     */
    Joints get_q() const;

    /**
     * @brief Gets the current joint velocities.
     *
     * Returns the latest velocities reported by the motors, from the last
     * feedback received.
     *
     * @return The joint velocities, in rad/s.
     */
    Joints get_dq() const;

    /**
     * @brief Gets the torque currently reported by each joint.
     *
     * Returns the latest torque values from the motor feedback, from the
     * last feedback received.
     *
     * @return The joint torques, in Nm.
     */
    Joints get_tau() const;

    /**
     * @brief Gets the MOSFET temperature of each motor driver.
     *
     * @return The MOSFET temperatures, in degrees Celsius.
     */
    Joints get_tmos() const;

    /**
     * @brief Gets the rotor (motor winding) temperature of each joint.
     *
     * @return The rotor temperatures, in degrees Celsius.
     */
    Joints get_trot() const;

    /**
     * @brief Opens the gripper to the default opening (80 %).
     */
    void open();

    /**
     * @brief Opens the gripper to a given width.
     *
     * @param meters Opening between the fingers, from 0.0 (closed) to 0.090
     *               (fully open). Throws if outside the gripper's range.
     */
    void open_width(double meters);

    /**
     * @brief Opens the gripper to a fraction of its full opening.
     *
     * @param ratio Normalized opening, from 0.0 (closed) to 1.0 (fully open).
     *              Throws if outside the gripper's range.
     */
    void open_ratio(double ratio);

    /**
     * @brief Closes the gripper.
     *
     * Closes it fully with the default grip strength.
     */
    void close();

    /**
     * @brief Closes the gripper with a chosen grip strength.
     *
     * @param f Desired gripping factor.
     *          0.0 for minimum closing strength.
     *          1.0 for maximum closing strength.
     */
    void close(double f);

    /**
     * @brief Prints an error message, including any errors nested inside it.
     *
     * Walks through the exception and any exceptions it wraps, printing
     * each one. Nested errors are indented further so you can follow the
     * chain from the outermost cause down to the root.
     *
     * @param e The exception to report.
     * @param level How deep in the chain we are, used for indentation. Leave it at 0 when calling it yourself.
     */
    void log_errors(const std::exception& e, int level = 0);

private:

    /**
     * @brief Finds the URDF file, following the lookup order of the constructor.
     *
     * @param urdf_path Path given by the user, or empty.
     * @return Path of the URDF file to load.
     */
    std::string resolve_urdf_path(const std::string& urdf_path);

    /**
     * @brief Enables a given set of motors.
     *
     * Sends the enable command to every motor in the list.
     *
     * @param motors The motors to enable.
     */
    void enable_motors(const std::vector<Motor>& motors);

    /**
     * @brief Disables a given set of motors.
     *
     * Sends the disable command to every motor in the list.
     *
     * @param motors The motors to disable.
     */
    void disable_motors(const std::vector<Motor>& motors);

    /**
     * @brief Checks that the motors run in MIT mode with the expected PMAX/VMAX/TMAX.
     *
     * @param motors The motors to check.
     */
    void verify_motor_config(const std::vector<Motor>& motors);

    /**
     * @brief Reads a register of a motor.
     *
     * @param motor The motor to read from.
     * @param rid The register to read.
     * @return The raw 4 bytes of the register.
     */
    uint32_t read_register(const Motor& motor, MotorRegister rid);

    /**
     * @brief Reads a float register and throws if it differs from the expected value.
     *
     * @param motor The motor to read from.
     * @param rid The register to read.
     * @param name Name of the register, used in the error message.
     * @param expected Expected value.
     */
    void check_register_value(const Motor& motor, MotorRegister rid, const std::string& name, double expected);

    /**
     * @brief Requests fresh feedback from a given set of motors.
     *
     * Sends a refresh request to every motor in the list, then reads
     * the replies back.
     *
     * @param motors The motors to refresh.
     */
    void refresh_motors(std::vector<Motor>& motors);

    /**
     * @brief Sends n frames to the serial port, one write per frame, without waiting for replies.
     *
     * Drops stale input first, so that only the replies to these frames are read.
     * Frames must not be merged into one write: the USB-CAN bridge drops frames
     * that straddle a USB packet boundary.
     *
     * @param frames The frames to send.
     * @param n Number of frames.
     */
    void send_frames(const SendFrame* frames, std::size_t n);

    /**
     * @brief Reads the feedback of every motor in the list.
     *
     * Reads frames until every motor has replied, matching them by master id,
     * skipping bridge status frames and frames from other motors.
     *
     * @param motors The motors that were just commanded.
     */
    void read_feedbacks(std::vector<Motor>& motors);

    /**
     * @brief Checks the feedback of a motor and stores its state.
     *
     * Throws on a motor fault, a wrong slave id, or a motor that reports
     * disabled while the arm is enabled.
     *
     * @param motor The motor the feedback belongs to.
     * @param payload The feedback payload.
     */
    void update_motor_state(Motor& motor, const Payload& payload);

    /**
     * @brief Computes the torque needed to cancel gravity.
     *
     * Uses the arm's dynamic model to work out how much torque each joint
     * must produce to hold the arm still in the given configuration.
     *
     * @param q Joint positions, in radians.
     * @return The gravity compensation torque for each joint, in N·m.
     */
    Joints compute_tau_gravity(const Joints& q);

    /**
     * @brief Checks that a command stays within the joint limits.
     *
     * Looks at the requested positions, velocities and torques to make sure
     * they're within what the joints can safely handle. This guards against
     * sending a command that could damage the arm.
     *
     * @param q_target Desired joint positions, in radians.
     * @param dq_target Desired joint velocities, in rad/s.
     * @param tau_target Desired joint torques, in N·m.
     * @param kp Position (stiffness) gains.
     * @param kd Velocity (damping) gains.
     */
    void check_manipulator_joint_limits(const Joints& q_target, const Joints& dq_target,
        const Joints& tau_target, const Joints& kp, const Joints& kd) const;

    /**
     * @brief Checks that position targets are close to the current position.
     *
     * Rejects targets further than max_q_step from the measured position on
     * joints with a non-zero stiffness, to avoid violent jumps. Large motions
     * must be interpolated by the caller.
     *
     * @param q_target Desired joint positions, in radians.
     * @param kp Position (stiffness) gains.
     */
    void check_position_step(const Joints& q_target, const Joints& kp) const;

    /**
     * @brief Checks that no motor is overheating.
     *
     * Prints a warning above warn_motor_temperature and throws above
     * max_motor_temperature. Uses the temperatures from the last feedback received.
     */
    void check_motor_temperatures();

    /**
     * @brief Throws if the motor data is older than max_motors_update_age.
     */
    void check_motor_data_age() const;

    /**
     * @brief Throws if the arm is not connected and enabled.
     */
    void check_robot_enabled() const;

    /**
     * @brief Throws if no gripper is mounted.
     */
    void check_gripper_mounted() const;

    /**
     * @brief Checks that a command stays within the joint limits.
     *
     * @param d_target Desired opening of end effector, in meters.
     */
    void check_gripper_joint_limits(double d_target) const;

    /**
     * @brief Converts a gripper opening into a motor position.
     *
     * Inverse kinematics for the gripper: given how far open you want it,
     * it works out the angle the gripper motor needs to reach.
     *
     * @param d Desired gripper opening in meters.
     * @return The matching gripper motor position, in radians.
     */
    double gripper_ik(double d) const;

    /**
     * @brief Sends a control command to the gripper motor.
     *
     * @param q Position target, in rad.
     * @param tau Feedforward torque, in Nm.
     * @param kp Position (stiffness) gain.
     * @param kd Velocity (damping) gain.
     */
    void control_gripper(double q, double tau, double kp, double kd);

    /**
     * @brief Whether a gripper is mounted on the arm.
     *
     * When false, the gripper functions and motors are ignored.
     */
    bool gripper;

    /**
     * @brief The arm's current state.
     */
    Status status;

    /**
     * @brief The motors driving the arm's joints.
     */
    std::vector<Motor> motors_manipulator;

    /**
     * @brief The motors driving the gripper.
     *
     * Only used when a gripper is mounted.
     */
    std::vector<Motor> motors_gripper;

    /**
     * @brief Handles the Damiao motor communication protocol.
     */
    DamiaoProtocol protocol;

    /**
     * @brief The serial port used to talk to the motors.
     */
    SerialPort port;

    /**
     * @brief The Pinocchio model of the arm, used for gravity compensation.
     */
    pinocchio::Model pin_model;

    /**
     * @brief Working data for the Pinocchio model.
     */
    pinocchio::Data pin_data;

    /**
     * @brief Full Pinocchio configuration, allocated once for the control loop.
     */
    Eigen::VectorXd pin_q;

    /**
     * @brief Position and velocity index of each manipulator joint in the Pinocchio model.
     */
    std::array<int, kNumJoints> joint_idx_q{};
    std::array<int, kNumJoints> joint_idx_v{};

    /**
     * @brief Position index of the gripper joint in the Pinocchio model.
     */
    int gripper_idx_q{};

    /**
     * @brief Timestamp of the most recent motor data update.
     */
    std::chrono::steady_clock::time_point motors_update_timestamp;

    /**
     * @brief True once the temperature warning was printed, until all motors cool down.
     */
    bool temperature_warning_printed{false};

    /**
     * @brief Maximum allowed age of motor data.
     */
    const std::chrono::milliseconds max_motors_update_age{500};

    /**
     * @brief Maximum distance between a position target and the current position.
     *
     * Applies to joints commanded with kp > 0. In radians.
     */
    const double max_q_step{0.2};

    /**
     * @brief MOSFET and rotor temperature that throws an error, in degrees Celsius.
     */
    const int max_motor_temperature{90};

    /**
     * @brief MOSFET and rotor temperature that prints a warning, in degrees Celsius.
     */
    const int warn_motor_temperature{70};

    /**
     * @brief Maximum opening of the gripper, in meters.
     */
    const double max_gripper_opening{0.090};

    /**
     * @brief Bridge status frames and frames from other motors skipped while reading feedback.
     */
    const std::size_t max_skipped_frames{8};
};

} // namespace ulixarm
