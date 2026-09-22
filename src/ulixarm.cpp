#include "ulixarm.hpp"

#include <iomanip>

UlixArm::UlixArm(bool gripper)
{
    this->status=IDLE;

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
    return;
}


uint32_t UlixArm::connect()
{
    uint32_t rc;

    // check status

    // open bus
    std::cout << "Opened bus" << std::endl;

    return rc;
}

uint32_t UlixArm::enable()
{
    uint32_t rc;

    std::cout << std::endl << "Enable motors..." << std::endl;
    
    // check status if connected
    

    for (const auto& motor : this->motors) {
        
        MotorID slave_id = motor.get_slave_id();
        
        // pack frame
        SendFrame send_frame;
        send_frame = protocol.encode_enable_command(slave_id);

        // send frame
        {
            for (const auto byte : send_frame.data) {
                std::cout   << std::hex
                << std::uppercase
                << std::setw(2)
                << std::setfill('0')
                << static_cast<unsigned int>(byte)
                << ' ';
            }

            std::cout << std::endl;
        }
    }

    return rc;
}

uint32_t UlixArm::disable()
{
    uint32_t rc;

    std::cout << std::endl << "Disable motors..." << std::endl;

    // check status

    
    for (const auto& motor : this->motors) {
        
        MotorID slave_id = motor.get_slave_id();
        
        // pack frame
        SendFrame send_frame;
        send_frame = protocol.encode_disable_command(slave_id);

        // send frame
        {
            for (const auto byte : send_frame.data) {
                std::cout   << std::hex
                << std::uppercase
                << std::setw(2)
                << std::setfill('0')
                << static_cast<unsigned int>(byte)
                << ' ';
            }

            std::cout << std::endl;
        }

    }

    return rc;
}

uint32_t UlixArm::control(
    joint_t q_target,
    joint_t dq_target,
    joint_t tau_target,
    joint_t kp,
    joint_t kd
)
{
    uint32_t rc;

    std::cout << std::endl << "Send commands..." << std::endl;

    // check status

    // for loop 1
    for (std::size_t i = 0; i < this->motors.size(); ++i) {
        const auto& motor = this->motors[i];

        // pack frame
        SendFrame send_frame;
        send_frame = protocol.encode_control_command(
            motor.get_slave_id(),
            motor.get_limit_param(),
            q_target(i),
            dq_target(i),
            tau_target(i),
            kp(i),
            kd(i)
        );

        // send frame
        {
            for (const auto byte : send_frame.data) {
                std::cout   << std::hex
                << std::uppercase
                << std::setw(2)
                << std::setfill('0')
                << static_cast<unsigned int>(byte)
                << ' ';
            }

            std::cout << std::endl;
        }

    }

    // pack frame

    // send frame


    // for loop 2

    // read frame

    // unpack frame

    return rc;
}

