#include "ulixarm.hpp"

#include <iomanip>

UlixArm::UlixArm(bool gripper)
{
    this->status = UlixArmStatus::IDLE;

    // create pinocchio model and data

    // add motors
    this->motors.emplace_back(0x01, 0x11, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}); // DM4340p
    this->motors.emplace_back(0x02, 0x12, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}); // DM4340
    this->motors.emplace_back(0x03, 0x13, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}); // DM4340
    this->motors.emplace_back(0x04, 0x14, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}); // DM4310
    this->motors.emplace_back(0x05, 0x15, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}); // DM4310
    this->motors.emplace_back(0x06, 0x16, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}); // DM4310

    if (gripper)
        this->motors.emplace_back(0x07, 0x17, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}); // DM4310

    std::cout << "Hello Ulix!" << std::endl;
    return;
}

UlixArm::~UlixArm()
{
    // close serial
    return;
}


uint32_t UlixArm::connect()
{
    uint32_t rc;

    // check status

    // open bus
    this->port.init("/dev/ttyACM0", 9600, 20);
    std::cout << "Opened bus" << std::endl;

    return rc;
}

uint32_t UlixArm::enable()
{
    uint32_t rc;

    std::cout << std::endl << "Enable motors..." << std::endl;
    
    // check status if connected
    

    for (const auto& motor : this->motors) {
        // pack frame
        SendFrame send_frame;
        send_frame = this->protocol.encode_enable_command(
            motor.get_slave_id()
        );

        // send frame
        this->port.write(send_frame.data, sizeof(send_frame.data));
    }

    return rc;
}

uint32_t UlixArm::disable()
{
    uint32_t rc;

    std::cout << std::endl << "Disable motors..." << std::endl;

    // check status

    
    for (const auto& motor : this->motors) {
        // pack frame
        SendFrame send_frame;
        send_frame = this->protocol.encode_disable_command(
            motor.get_slave_id()
        );
        
        // send frame
        this->port.write(send_frame.data, sizeof(send_frame.data));
    }

    return rc;
}

uint32_t UlixArm::control(
    Joints q_target,
    Joints dq_target,
    Joints tau_target,
    Joints kp,
    Joints kd,
    bool gravity
)
{
    uint32_t rc;

    std::cout << std::endl << "Send commands..." << std::endl;

    // check status


    // compute tau gravity
    if (gravity)
    {
        // pinocchio   
    }

    // send commands
    for (std::size_t i = 0; i < this->motors.size(); ++i) {
        const auto& motor = this->motors[i];

        // pack frame
        SendFrame send_frame;
        send_frame = this->protocol.encode_control_command(
            motor.get_slave_id(),
            motor.get_limit_param(),
            q_target(i),
            dq_target(i),
            tau_target(i),
            kp(i),
            kd(i)
        );

        // send frame
        this->port.write(send_frame.data, sizeof(send_frame.data));

        // // send frame
        // {
        //     for (const auto byte : send_frame.data) {
        //         std::cout   << std::hex
        //         << std::uppercase
        //         << std::setw(2)
        //         << std::setfill('0')
        //         << static_cast<unsigned int>(byte)
        //         << ' ';
        //     }

        //     std::cout << std::endl;
        // }

    }


    // read feedback
    for (std::size_t i = 0; i < this->motors.size(); ++i) {
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
        uint8_t motor_id = master_id & 0x0F;
        Motor& motor = this->motors[motor_id];
        MotorLimitParam limit_param = motor.get_limit_param();
        double q, dq, tau;
        int trot, tmos;
        this->protocol.decode_control_feedback(
            payload,
            limit_param,
            q,
            dq,
            tau,
            trot,
            tmos            
        );
        
        // update internal state
        motor.set_q(q);
        motor.set_dq(dq);
        motor.set_tau(tau);
        motor.set_trot(trot);
        motor.set_tmos(tmos);

    }

    return rc;
}

