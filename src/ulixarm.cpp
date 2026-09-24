#include "ulixarm_cpp/ulixarm.hpp"

#include <iomanip>


UlixArm::UlixArm(bool gripper)
{
    this->status = UlixArmStatus::IDLE;

    // create pinocchio model and data

    // add motors
    this->motors.emplace_back(0x01, 0x11, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}); // DM4340p
    this->motors.emplace_back(0x02, 0x12, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}); // DM4340
    // this->motors.emplace_back(0x03, 0x13, MotorLimitParam{.q = 12.5, .dq =  8.0, .tau = 28.0}); // DM4340
    // this->motors.emplace_back(0x04, 0x14, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}); // DM4310
    this->motors.emplace_back(0x05, 0x15, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}); // DM4310
    // this->motors.emplace_back(0x06, 0x16, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}); // DM4310

    // if (gripper)
    //     this->motors.emplace_back(0x07, 0x17, MotorLimitParam{.q = 12.5, .dq = 30.0, .tau = 10.0}); // DM4310

    return;
}

UlixArm::~UlixArm()
{
    // close serial
    return;
}


int UlixArm::connect()
{
    int rc;

    // check status

    // open bus
    rc = this->port.init("/dev/ttyACM0", 115200, 20);
    if (rc < 0)
    {
        std::cout << "Failed to open bus" << std::endl;
        return rc;
    }        

    return rc;
}

int UlixArm::enable()
{
    int rc = 1;

    // check status if connected
    
    for (const auto& motor : this->motors) {
        // pack frame
        SendFrame send_frame;
        send_frame = this->protocol.encode_enable_command(
            motor.get_slave_id()
        );

        // send frame
        rc = this->port.write(send_frame.data, sizeof(send_frame.data));
    }

    return rc;
}

int UlixArm::disable()
{
    int rc;

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

int UlixArm::refresh()
{
    int rc;

    // check status
    
    for (const auto& motor : this->motors) {
        // pack frame
        SendFrame send_frame;
        send_frame = this->protocol.encode_refresh_command(
            motor.get_slave_id()
        );
        
        // send frame
        this->port.write(send_frame.data, sizeof(send_frame.data));
    }

    this->update_status();

    return rc;
}

int UlixArm::control(
    Joints q_target,
    Joints dq_target,
    Joints tau_target,
    Joints kp,
    Joints kd,
    bool gravity
)
{
    std::cout << std::endl;

    int rc;

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

        std::cout << "Target:" << std::endl;
        std::cout << "q: " << q_target(i) << " dq: " << dq_target(i) <<  " tau: " << tau_target(i) <<  std::endl;

        // send frame
        this->port.write(send_frame.data, sizeof(send_frame.data));
    }

    this->update_status();

    return rc;
}

void UlixArm::update_status()
{
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
}

Joints UlixArm::get_q()
{
    Joints q = Joints::Zero();;
    for (std::size_t i = 0; i < this->motors.size(); ++i) {
        q(i) = this->motors[i].get_q();
    }
    return q;
}

Joints UlixArm::get_dq()
{
    Joints dq = Joints::Zero();
    for (std::size_t i = 0; i < this->motors.size(); ++i) {
        dq(i) = this->motors[i].get_dq();
    }
    return dq;
}

Joints UlixArm::get_tau()
{
    Joints tau = Joints::Zero();
    for (std::size_t i = 0; i < this->motors.size(); ++i) {
        tau(i) = this->motors[i].get_tau();
    }
    return tau;
}

Joints UlixArm::get_tmos()
{
    Joints tmos = Joints::Zero();
    for (std::size_t i = 0; i < this->motors.size(); ++i) {
        tmos(i) = static_cast<double>(this->motors[i].get_tmos());
    }
    return tmos;
}

Joints UlixArm::get_trot()
{
    Joints trot = Joints::Zero();
    for (std::size_t i = 0; i < this->motors.size(); ++i) {
        trot(i) = static_cast<double>(this->motors[i].get_trot());
    }
    return trot;
}


Motor* UlixArm::find_motor_by_master_id(MotorID master_id)
{
    for (auto& motor : this->motors) {
        if (motor.get_master_id() == master_id) {
            return &motor;
        }
    }

    return nullptr;
}
