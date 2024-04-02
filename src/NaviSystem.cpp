#include "NaviSystem.hpp"
#include <functional>
#include <iostream>
#include "Utilities/Converters.h"


NaviSystem::NaviSystem(zmq::context_t& ctx, const Config& config)
    :   PeriodicEvent(config.loop_rate_ms, false),
        udp_listener{config.sensor_multicast_address, config.sensor_multicast_port, config.sensor_multicast_interface},
        time_synchronizer(config.time_sync_period_ms, config.time_sync_address, config.time_sync_port, config.sensor_multicast_interface),
        ekf(config.drawbar_length),
        ekf_logger("ekf", "time,x,y,z,vx,vy,vz,q0,qx,qy,qz,gyro_bias_X,gyro_bias_Y,gyro_bias_Z,az,el"),
        accelerometer(time_synchronizer,!config.accelerometer_calibration),
        gyroscope(time_synchronizer,false),
        position_provider(time_synchronizer),
        fanuc_position(time_synchronizer),
        ctx{ctx},
        debug_mode{config.debug_mode}

{
    status_sock = zmq::socket_t(ctx, zmq::socket_type::pub);
    status_sock.bind(status_address);
    set_EKF_parameters();
    udp_listener.set_message_event(std::bind(&NaviSystem::messageHandler, this, std::placeholders::_1));
    start_periodic_task();
}

NaviSystem::~NaviSystem() 
{
    udp_listener.stop_listening_thread();
    status_sock.close();
}

void NaviSystem::periodic_event() 
{
    static uint8_t counter = 1;
    if(debug_mode)
    {
        auto time = Millis::get();
        std::optional<Eigen::Vector3f> position = std::nullopt;
        if(position_provider.has_new_value())
        {
            position = position_provider.get_value(true).second;
        }
        ekf.update(time, Eigen::Vector3f::Zero(),
            Eigen::Vector3f{0.0f, 0.0f, 9.805f},
            position);
        ekf_logger(time, ekf.get_state());
        send_status();
        return;
    }
    if(accelerometer.has_new_value() && gyroscope.has_new_value())
    {
        auto accelerometer_reading = accelerometer.get_value(true);
        auto gyroscope_reading = gyroscope.get_value(true);
        auto time = (accelerometer_reading.first + gyroscope_reading.first) / 2.0f;
        if(fanuc_position.has_new_value())
        {
            ekf.update(time,
                       Converters::mdeg_to_radians(gyroscope_reading.second),
                       accelerometer_reading.second,
                       fanuc_position.get_value(true).second);
        }
        else
        {
            ekf.update(time, Converters::mdeg_to_radians(gyroscope_reading.second), accelerometer_reading.second);
        }
        ekf_logger(time, ekf.get_state());
        if(counter++ % 5 == 0)
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
        case Command::POSITION_READING:
            position_provider.consumeMessage(msg);
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

    auto forces = force_logger.get_forces();
    if (forces.size() > 1)
    {
        ss << "f:" << forces.format(commaFormat);
        s = ss.str();
        message.rebuild(s.data(), s.size());
        ss.str("");
        status_sock.send(message,zmq::send_flags::none);
    }
}

void NaviSystem::set_EKF_parameters() 
{
    auto config = Config::get_singleton();
    ekf.set_position_process_noise(config.position_process_noise);
    ekf.set_velocity_process_noise(config.velocity_process_noise);
    ekf.set_quaterion_process_noise(config.quaterion_process_noise);
    ekf.set_gyro_bias_process_noise(config.gyro_bias_process_noise);
    ekf.set_drawbar_process_noise(config.drawbar_process_noise);
    ekf.set_accel_measurement_noise(config.accel_measurement_noise);
    ekf.set_pos_provider_measurement_noise(config.pos_provider_measurement_noise);
    ekf.set_constraint_correction_scaler(config.constraint_correction_scaler);
    ekf.set_constraint_correction_repeats(config.constraint_correction_repeats);
}
