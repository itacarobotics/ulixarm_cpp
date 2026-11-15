#pragma once

#include <cstdint>

#include "motor.hpp"

namespace ulixarm
{

struct SendFrame
{
    uint8_t data[30]{};
};

// frames are written back to back in a single write
static_assert(sizeof(SendFrame) == 30, "SendFrame must not be padded");

struct ReceiveFrame
{
    uint8_t data[16]{};
};

struct Payload
{
    uint8_t data[8]{};
};

// Damiao register IDs (RID) used by the library
enum class MotorRegister : uint8_t
{
    CTRL_MODE = 10, // uint32: 1 = MIT, 2 = position-velocity, 3 = velocity, 4 = position-force
    PMAX = 21,      // float: position mapping range [rad]
    VMAX = 22,      // float: velocity mapping range [rad/s]
    TMAX = 23,      // float: torque mapping range [Nm]
};

constexpr uint32_t ctrl_mode_mit = 1;

class DamiaoProtocol
{
public:
    // gain ranges of the MIT command
    static constexpr double kp_max = 500.0;
    static constexpr double kd_max = 5.0;

    static constexpr uint8_t head = 0xAA;
    static constexpr uint8_t tail = 0x55;

    // write
    SendFrame build_usb_bridge_frame(MotorID can_id, const Payload& can_payload) const;

    SendFrame encode_enable_command(MotorID slave_id) const;
    SendFrame encode_disable_command(MotorID slave_id) const;
    SendFrame encode_refresh_command(MotorID slave_id) const;
    SendFrame encode_read_register_command(MotorID slave_id, MotorRegister rid) const;
    SendFrame encode_control_command(MotorID slave_id, const MotorLimitParam& limit_param,
        double q, double dq, double tau, double kp, double kd) const;

    // read
    // returns true if the frame carries CAN data, false for bridge status frames (heartbeat, send ack)
    bool parse_usb_bridge_frame(const ReceiveFrame& frame, MotorID &master_id, Payload &payload) const;

    // throws on a motor fault, or if expect_enabled and the motor reports disabled
    void check_motor_state(const Payload& payload, bool expect_enabled) const;

    // slave id carried in the low nibble of a feedback payload
    static MotorID feedback_slave_id(const Payload& payload) { return payload.data[0] & 0x0F; }

    void decode_control_feedback(const Payload& payload, const MotorLimitParam& limit_param,
        double &q, double &dq, double &tau, int &trot, int &tmos) const;

    // returns true if payload is the reply to a register read of (slave_id, rid), raw = register bytes
    bool decode_register_reply(const Payload& payload, MotorID slave_id, MotorRegister rid, uint32_t &raw) const;

private:
    static uint16_t double_to_uint(double x, double xmin, double xmax, uint8_t bits);
    static double   uint_to_double(uint16_t x, double xmin, double xmax, uint8_t bits);
};

} // namespace ulixarm
