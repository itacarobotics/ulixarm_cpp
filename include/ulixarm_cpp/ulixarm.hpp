#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <exception>
#include <memory>
#include <string>

namespace ulixarm
{

constexpr std::size_t kNumJoints = 6;

using Joints = Eigen::Matrix<double, kNumJoints, 1>;

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

    UlixArm(const UlixArm&) = delete;
    UlixArm& operator=(const UlixArm&) = delete;

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
     * kp > 0 must be within 0.2 rad of their current position, and no motor
     * may exceed 90 C (a warning is printed from 70 C).
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
     * the last feedback received, from refresh() or control(). Throws if
     * that feedback is older than 500 ms.
     *
     * @return The joint angles, in radians.
     */
    Joints get_q() const;

    /**
     * @brief Gets the current joint velocities, in rad/s. See get_q().
     */
    Joints get_dq() const;

    /**
     * @brief Gets the torque currently reported by each joint, in Nm. See get_q().
     */
    Joints get_tau() const;

    /**
     * @brief Gets the MOSFET temperature of each motor driver, in degrees Celsius. See get_q().
     */
    Joints get_tmos() const;

    /**
     * @brief Gets the rotor (motor winding) temperature of each joint, in degrees Celsius. See get_q().
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
    static void log_errors(const std::exception& e, int level = 0);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

} // namespace ulixarm
