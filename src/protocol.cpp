#include "protocol.hpp"

#include <algorithm>

SendFrame DamiaoProtocol::build_usb_bridge_frame(MotorID can_id, Payload can_payload)
{
    SendFrame send_frame;
    
    // header
    send_frame.data[0]  = 0x55;
    send_frame.data[1]  = 0xAA;
    
    // frame length
    send_frame.data[2]  = 0x1e;
    
    // command: 1 = forward CAN data frame
    //          2 = PC–device handshake, device responds OK
    //          3 = non-acknowledged CAN forward, no send-status feedback
    send_frame.data[3]  = 0x03;
    
    // send times (1)
    send_frame.data[4]  = 0x01;
    send_frame.data[5]  = 0x00;
    send_frame.data[6]  = 0x00;
    send_frame.data[7]  = 0x00;

    // send interval (10 ms) - NOTE: not used for send times 1
    send_frame.data[8]  = 0x0A;
    send_frame.data[9]  = 0x00;
    send_frame.data[10] = 0x00;
    send_frame.data[11] = 0x00;
    
    // id type:  0 = standard frame  1 = extended frame 
    send_frame.data[12] = 0x00;
    
    // slave id
    send_frame.data[13] = can_id & 0xFF;
    send_frame.data[14] = (can_id >> 8) & 0xFF;
    send_frame.data[15] = (can_id >> 16) & 0xFF;
    send_frame.data[16] = (can_id >> 24) & 0xFF;
    
    // frame type: 0 = data frame  1 = remote frame
    send_frame.data[17] = 0x00;
    
    // payload length
    send_frame.data[18] = 0x08;

    // id acc (?)
    send_frame.data[19] = 0x00;

    // data acc (?)
    send_frame.data[20] = 0x00;

    // payload
    send_frame.data[21] = can_payload.data[0];
    send_frame.data[22] = can_payload.data[1];
    send_frame.data[23] = can_payload.data[2];
    send_frame.data[24] = can_payload.data[3];
    send_frame.data[25] = can_payload.data[4];
    send_frame.data[26] = can_payload.data[5];
    send_frame.data[27] = can_payload.data[6];
    send_frame.data[28] = can_payload.data[7];
    
    // crc
    send_frame.data[29] = 0x00;

    return send_frame;

}

SendFrame DamiaoProtocol::encode_enable_command(MotorID slave_id)
{
    // build can data
    Payload payload;
    payload.data[0] = 0xFF;
    payload.data[1] = 0xFF;
    payload.data[2] = 0xFF;
    payload.data[3] = 0xFF;
    payload.data[4] = 0xFF;
    payload.data[5] = 0xFF;
    payload.data[6] = 0xFF;
    payload.data[7] = 0xFC; // enable

    return this->build_usb_bridge_frame(slave_id, payload);
}

SendFrame DamiaoProtocol::encode_disable_command(MotorID slave_id)
{
    // build can data
    Payload payload;
    payload.data[0] = 0xFF;
    payload.data[1] = 0xFF;
    payload.data[2] = 0xFF;
    payload.data[3] = 0xFF;
    payload.data[4] = 0xFF;
    payload.data[5] = 0xFF;
    payload.data[6] = 0xFF;
    payload.data[7] = 0xFD; // disable

    return this->build_usb_bridge_frame(slave_id, payload);
}

SendFrame DamiaoProtocol::encode_refresh_command(MotorID slave_id)
{
    MotorID query_id = 0x7FF;

    // build can data
    Payload payload;
    payload.data[0] = slave_id & 0xFF;
    payload.data[1] = (slave_id >> 8) & 0xFF;
    payload.data[2] = 0xCC;
    payload.data[3] = 0x00;
    payload.data[4] = 0x00;
    payload.data[5] = 0x00;
    payload.data[6] = 0x00;
    payload.data[7] = 0x00;

    return this->build_usb_bridge_frame(query_id, payload);
}


SendFrame DamiaoProtocol::encode_control_command(MotorID slave_id, MotorLimitParam limit_param, 
    double q, double dq, double tau, double kp, double kd)
{
    // map linearly to given bounds
    uint16_t q_uint = double_to_uint(q, -limit_param.q, limit_param.q, 16);
    uint16_t dq_uint = double_to_uint(dq, -limit_param.dq, limit_param.dq, 12);
    uint16_t tau_uint = double_to_uint(tau, -limit_param.tau, limit_param.tau, 12);
    uint16_t kp_uint = double_to_uint(kp, 0, 500, 12);
    uint16_t kd_uint = double_to_uint(kd, 0, 5, 12);

    // build can data
    Payload payload;
    payload.data[0] = static_cast<uint8_t>((q_uint >> 8) & 0xff);
    payload.data[1] = static_cast<uint8_t>(q_uint & 0xff);
    payload.data[2] = static_cast<uint8_t>(dq_uint >> 4);
    payload.data[3] = static_cast<uint8_t>(((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf));
    payload.data[4] = static_cast<uint8_t>(kp_uint & 0xff);
    payload.data[5] = static_cast<uint8_t>(kd_uint >> 4);
    payload.data[6] = static_cast<uint8_t>(((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf));
    payload.data[7] = static_cast<uint8_t>(tau_uint & 0xff);

    return this->build_usb_bridge_frame(slave_id, payload);
}

// read
void DamiaoProtocol::parse_usb_bridge_frame(ReceiveFrame frame, MotorID &master_id, Payload &payload)
{
    // frame.data[0] -> header
    
    // Command  0x00: heartbeat
    //          0x01: receive fail  
    //          0x11: receive success
    //          0x02: send fail
    //          0x12: send success
    //          0x03: set baudrate fail 0x13: set baudrate success
    //          0xEE: communication error, format field contains error code
    //          0x08: overvoltage
    //          0x09: undervoltage
    //          0x0A: overcurrent
    //          0x0B: MOS overtemperature
    //          0x0C: motor coil overtemperature
    //          0x0D: communication loss
    //          0x0E: overload
    uint8_t command = frame.data[1];

    switch (command)
    {
    case 0x00:
        // Heartbeat
        break;
    case 0x01:
        throw std::runtime_error(
            "Motor communication error: receive failed."
        );
    case 0x11:
        // Receive success
        break;
    case 0x02:
        throw std::runtime_error(
            "Motor communication error: send failed."
        );
    case 0x12:
        // Send success
        break;
    case 0x03:
        throw std::runtime_error(
            "Motor communication error: failed to set baudrate."
        );
    case 0x13:
        // Set baudrate success
        break;
    case 0xEE:
        throw std::runtime_error(
            "Motor communication error: 0xEE"
        );
    case 0x08:
        throw std::runtime_error(
            "Motor error: overvoltage."
        );
    case 0x09:
        throw std::runtime_error(
            "Motor error: undervoltage."
        );
    case 0x0A:
        throw std::runtime_error(
            "Motor error: overcurrent."
        );
    case 0x0B:
        throw std::runtime_error(
            "Motor error: MOS overtemperature."
        );
    case 0x0C:
        throw std::runtime_error(
            "Motor error: motor coil overtemperature."
        );
    case 0x0D:
        throw std::runtime_error(
            "Motor error: communication loss."
        );
    case 0x0E:
        throw std::runtime_error(
            "Motor error: overload."
        );
    default:
        throw std::runtime_error(
            "Motor communication error: unknown command 0x" +
            std::to_string(command)
        );
    }

    // frame.data[2] -> can data length
    // frame.data[3] -> can ide     0: standard frame    1: extended frame
    // frame.data[4] -> can rtr     0: data frame        1: remote frame
    
    // Master ID fed back from the motor
    master_id = static_cast<uint32_t>(frame.data[5])
             | (static_cast<uint32_t>(frame.data[6]) << 8)
             | (static_cast<uint32_t>(frame.data[7]) << 16)
             | (static_cast<uint32_t>(frame.data[8]) << 24);
    
    // can data
    payload.data[0] = frame.data[ 9];
    payload.data[1] = frame.data[10];
    payload.data[2] = frame.data[11];
    payload.data[3] = frame.data[12];
    payload.data[4] = frame.data[13];
    payload.data[5] = frame.data[14];
    payload.data[6] = frame.data[15];
    payload.data[7] = frame.data[16];
    
    // frame.data[17] -> tail
    
    return;
}

void DamiaoProtocol::decode_control_feedback(Payload payload, MotorLimitParam limit_param, double &q, double &dq, double &tau, int &trot, int &tmos)
{
    uint16_t q_uint = (uint16_t(payload.data[1]) << 8) | payload.data[2];
    uint16_t dq_uint = (uint16_t(payload.data[3]) << 4) | (payload.data[4] >> 4);
    uint16_t tau_uint = (uint16_t(payload.data[4] & 0xf) << 8) | payload.data[5];
    
    q   = uint_to_double(q_uint, -limit_param.q, limit_param.q, 16);
    dq  = uint_to_double(dq_uint, -limit_param.dq, limit_param.dq, 12);
    tau = uint_to_double(tau_uint, -limit_param.tau, limit_param.tau, 12);
    
    tmos = static_cast<int>(payload.data[6]);
    trot = static_cast<int>(payload.data[7]);

    return;
}


uint16_t DamiaoProtocol::double_to_uint(double x, double xmin, double xmax, uint8_t bits)
{
    double span = xmax - xmin;
    double data_norm = std::clamp((x - xmin) / span, 0.0, 1.0);
    return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
}

double DamiaoProtocol::uint_to_double(uint16_t x, double xmin, double xmax, uint8_t bits)
{
    double span = xmax - xmin;
    double data_norm = static_cast<double>(x) / ((1 << bits) - 1);
    double data = data_norm * span + xmin;
    return data;
}
