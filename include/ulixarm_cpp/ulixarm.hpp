#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <stdint.h>
#include <vector>
#include <iomanip>
#include <chrono>
#include <cmath>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/collision/collision.hpp>


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

    /**
     * @brief Creates the arm object and sets up everything it needs to run.
     *
     * Loads the kinematic/dynamic model used for gravity compensation and
     * prepares the motor lists for the manipulator joints and, if present,
     * the gripper. It doesn't talk to the hardware yet; call connect()
     * for that.
     *
     * @param gripper Set to true if a gripper is mounted on the arm, false if not.
     */
    UlixArm(bool gripper=true);
    
    /**
     * @brief Cleans up when the arm object goes away.
     *
     * Releases the resources the arm was holding, such as the serial
     * connection to the motors.
     */
    ~UlixArm();

    /**
     * @brief Opens the serial connection to the arm.
     *
     * Connects to the motor controller on the given port. This has to
     * happen before enabling the motors or sending any commands.
     *
     * @param port Path of the serial device, for example "/dev/ttyACM0".
     */
    void connect(const char* port);

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
     * 
     * @note Use this function at initialization only, as the control() function, 
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
     * @note This method does not control the gripper, use open() and close() instead.
     * 
     * @warning Self collision is NOT checked yet.
     *
     * @param q_target Desired joint positions, in rad.
     * @param dq_target Desired joint velocities, in rad/s.
     * @param tau_target Feedforward torques, in Nm.
     * @param kp Position (stiffness) gains, one per joint.
     * @param kd Velocity (damping) gains, one per joint.
     * @param gravity True to add gravity compensation, false to leave it out.
     */
    void control(
        Joints q_target,
        Joints dq_target,
        Joints tau_target,
        Joints kp,
        Joints kd,
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
    Joints get_q();

    /**
     * @brief Gets the current joint velocities.
     *
     * Returns the latest velocities reported by the motors, from the last
     * feedback received.
     *
     * @return The joint velocities, in rad/s.
     */
    Joints get_dq();

    /**
     * @brief Gets the torque currently reported by each joint.
     *
     * Returns the latest torque values from the motor feedback, from the
     * last feedback received.
     *
     * @return The joint torques, in Nm.
     */
    Joints get_tau();

    /**
     * @brief Gets the MOSFET temperature of each motor driver.
     *
     * Handy for keeping an eye on the drivers so they don't overheat
     * during long or demanding motions.
     *
     * @return The MOSFET temperatures, in degrees Celsius.
     */
    Joints get_tmos();

    /**
     * @brief Gets the rotor (motor winding) temperature of each joint.
     *
     * Useful for spotting motors that are working too hard before they
     * reach their thermal limits.
     *
     * @return The rotor temperatures, in degrees Celsius.
     */
    Joints get_trot();

    /**
     * @brief Opens the gripper all the way.
     */
    void open();

    /**
     * @brief Opens the gripper to a specific width.
     *
     * @param d Desired opening of the gripper in meters. Values beyond what the
     *          gripper can physically do are limited to its range. 
     *          Minimum opening is 0.0 meters.
     *          Maximum opening is 0.090 meters.
     */
    void open(double d);

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
     * @brief Requests fresh feedback from a given set of motors.
     *
     * Asks every motor in the list to report its current state, then reads
     * the replies back.
     *
     * @param motors The motors to refresh.
     */
    void refresh_motors(const std::vector<Motor>& motors);

    /**
     * @brief Looks up a motor by its master ID.
     *
     * Used to match an incoming feedback message with the motor it came from.
     *
     * @param master_id The master ID reported in the message.
     * @return A pointer to the matching motor, or nullptr if none matches.
     */
    Motor* find_motor_by_master_id(MotorID master_id);
    
    /**
     * @brief Reads incoming feedback from the serial port.
     *
     * Decodes the messages sent back by the motors and updates the stored
     * state (position, velocity, torque and temperatures) of the matching motors.
     */
    void read_feedback();

    /**
     * @brief Computes the torque needed to cancel gravity.
     *
     * Uses the arm's dynamic model to work out how much torque each joint
     * must produce to hold the arm still in the given configuration.
     *
     * @param q Joint positions, in radians.
     * @return The gravity compensation torque for each joint, in N·m.
     */
    Joints compute_tau_gravity(Joints q);

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
    void check_manipulator_joint_limits(Joints q_target, Joints dq_target, Joints tau_target, Joints kp, Joints kd);

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
    void check_position_step(Joints q_target, Joints kp);

    /**
     * @brief Checks that no motor is overheating.
     *
     * Uses the temperatures from the last feedback received.
     */
    void check_motor_temperatures();

    /**
     * @brief Throws if the arm is not connected and enabled.
     */
    void check_enabled();

    /**
     * @brief Throws if no gripper is mounted.
     */
    void check_gripper_mounted();

    /**
     * @brief Checks that a command stays within the joint limits.
     *
     * @param d_target Desired opening of end effector, in meters.
     */
    void check_gripper_joint_limits(double d_target);

    /**
     * @brief Checks for self collisions.
     *
     * @param q_target Desired joint positions, in radians.
     */
    void check_self_collision(Joints q_target);

    /**
     * @brief Converts a gripper opening into a motor position.
     *
     * Inverse kinematics for the gripper: given how far open you want it,
     * it works out the angle the gripper motor needs to reach.
     *
     * @param d Desired gripper opening in meters.
     * @return The matching gripper motor position, in radians.
     */
    double gripper_ik(double d);

    /**
     * @brief Whether a gripper is mounted on the arm.
     *
     * When false, the gripper functions, gravity compensation and motors are ignored.
     */
    bool gripper;

    /**
     * @brief The arm's current state.
     *
     * Tracks where the arm is in its lifecycle, from idle and connected
     * through enabled, disabled and running, or error if something went wrong.
     */
    UlixArmStatus status;

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
     *
     * Builds the commands sent to the motors and decodes their replies.
     */
    DamiaoProtocol protocol;

    /**
     * @brief The serial port used to talk to the motors.
     */
    SerialPort port;

    /**
     * @brief The Pinocchio model of the arm.
     *
     * Describes the arm's links and joints, and is used for dynamics
     * calculations such as gravity compensation.
     */
    pinocchio::Model pin_model;

    /**
     * @brief Working data for the Pinocchio model.
     *
     * Holds the intermediate results of the dynamics calculations done
     * with pin_model.
     */
    pinocchio::Data pin_data;

    pinocchio::GeometryModel pin_collision_model;
    pinocchio::GeometryData pin_collision_data;

    /**
     * @brief Timestamp of the most recent motor data update.
     *
     * Used to determine the age of the motor data and prevent consuming
     * stale measurements.
     */
    std::chrono::steady_clock::time_point motors_update_timestamp;

    /**
     * @brief Maximum allowed age of motor data.
     *
     * Motor data older than this duration is considered stale and cannot
     * be consumed. Defaults to 100 milliseconds.
     */
    const std::chrono::milliseconds max_motors_update_age{100};

    /**
     * @brief Maximum distance between a position target and the current position.
     *
     * Applies to joints commanded with kp > 0. In radians.
     */
    const double max_q_step{0.2};

    /**
     * @brief Maximum allowed MOSFET and rotor temperature, in degrees Celsius.
     */
    const int max_motor_temperature{80};

};
