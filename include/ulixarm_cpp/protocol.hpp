#ifndef ULIXARM_CPP_PROTOCOL_HPP
#define ULIXARM_CPP_PROTOCOL_HPP

#include <iostream>
#include <stdint.h>

#include "motor.hpp"

struct SendFrame
{
    uint8_t data[29]{};
};

struct ReceiveFrame
{
    uint8_t data[18]{};
};

struct Payload
{
    uint8_t data[8]{};
};

class DamiaoProtocol
{
public:
    DamiaoProtocol() {};
    ~DamiaoProtocol() {};

    // write
    SendFrame build_usb_bridge_frame(MotorID can_id, Payload can_payload);
    
    SendFrame encode_enable_command(MotorID slave_id);
    SendFrame encode_disable_command(MotorID slave_id);
    SendFrame encode_refresh_command(MotorID slave_id);
    SendFrame encode_control_command(MotorID slave_id, MotorLimitParam limit_param, 
        double q, double dq, double tau, double kp, double kd);

    // read
    void parse_usb_bridge_frame(ReceiveFrame frame, MotorID &master_id, Payload &payload);
    void decode_control_feedback(Payload payload, MotorLimitParam limit_param, double &q, double &dq, double &tau, int &trot, int &tmos);
    
    uint8_t head = 0xAA;
    uint8_t tail = 0x55;

private:
    
    uint16_t double_to_uint(double x, double xmin, double xmax, uint8_t bits);
    double   uint_to_double(uint16_t x, double xmin, double xmax, uint8_t bits);

};


#endif // ULIXARM_CPP_PROTOCOL_HPP