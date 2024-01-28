#include "NaviSystem.hpp"
#include <functional>
#include <iostream>

NaviSystem::NaviSystem()
    :   PeriodicEvent(20, false),
        udp_listener{1234},
        time_synchronizer(5000,50000),
        accelerometer(time_synchronizer),
        gyroscope(time_synchronizer)

{
    status_sock = zmq::socket_t(ctx, zmq::socket_type::pub);
    status_sock.bind(status_address);

    udp_listener.set_message_event(std::bind(&NaviSystem::messageHandler, this, std::placeholders::_1));
    start_periodic_task();
}

NaviSystem::~NaviSystem() 
{

}

void NaviSystem::periodic_event() 
{
    send_status();
}

void NaviSystem::messageHandler(const Message &msg) 
{
    switch (msg.command)
    {
        case Command::TIMESYNC:
            time_synchronizer.consumeMessage(msg);
            return;
        case Command::ACCELEROMETER_READING:
            accelerometer.consumeMessage(msg);
            return;
        case Command::GYROSCOPE_READING:
            gyroscope.consumeMessage(msg);
            return;
        default:
            return;
    }

}

void NaviSystem::send_status() 
{
    static Eigen::IOFormat commaFormat(3, Eigen::DontAlignCols," ",",");
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    zmq::message_t message;

    if(accelerometer.healthy())
    {
        auto reading = accelerometer.get_value();
        ss << "a:"  << reading.first << "," << reading.second.format(commaFormat);
        s = ss.str();
        //std::cout << s << std::endl;
        message.rebuild(s.data(), s.size());
        ss.str("");
        status_sock.send(message,zmq::send_flags::none);
    }

    if(gyroscope.healthy())
    {
        auto reading = gyroscope.get_value();
        ss << "g:"  << reading.first << "," << reading.second.format(commaFormat);
        s = ss.str();
        //std::cout << s << std::endl;
        message.rebuild(s.data(), s.size());
        ss.str("");
        status_sock.send(message,zmq::send_flags::none);
    }
}
