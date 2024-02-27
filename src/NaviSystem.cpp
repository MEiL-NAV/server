#include "NaviSystem.hpp"
#include <functional>
#include <iostream>
#include "Utilities/Converters.h"

NaviSystem::NaviSystem()
    :   PeriodicEvent(5, false),
        udp_listener{1234},
        time_synchronizer(5000,50000),
        accelerometer(time_synchronizer,true),
        gyroscope(time_synchronizer,false)

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
    static uint8_t counter = 1;
    if(accelerometer.healthy() && gyroscope.healthy())
    {
        auto accelerometer_reading = accelerometer.get_value();
        auto gyroscope_reading = gyroscope.get_value();
        auto time = (accelerometer_reading.first + gyroscope_reading.first) / 2.0f;
        ekf.update(time, Converters::mdeg_to_radians(gyroscope_reading.second), accelerometer_reading.second);
        if(counter++ % 10 == 0)
        {
            send_status();
        }
    }
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
        message.rebuild(s.data(), s.size());
        ss.str("");
        status_sock.send(message,zmq::send_flags::none);
    }

    if(gyroscope.healthy())
    {
        auto reading = gyroscope.get_value();
        ss << "g:"  << reading.first << "," << reading.second.format(commaFormat);
        s = ss.str();
        message.rebuild(s.data(), s.size());
        ss.str("");
        status_sock.send(message,zmq::send_flags::none);
    }

    ss << "s:" << ekf.get_state().format(commaFormat);
    s = ss.str();
    message.rebuild(s.data(), s.size());
    ss.str("");
    status_sock.send(message,zmq::send_flags::none);

}
