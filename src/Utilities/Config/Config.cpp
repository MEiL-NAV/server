#include "Config.h"
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "../Loggers/Logger.h"
#include "yaml_extender.h"

std::unique_ptr<Config> Config::singleton = nullptr;

Config::Config(const char *config_file_path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(config_file_path);
        log_mask = config["log_mask"].as<int>();
        log_path = config["log_path"].as<std::string>();
        run_counter_path = config["run_counter_path"].as<std::string>();
        logger_address = config["logger_address"].as<std::string>();

        sensor_multicast_address = config["sensor_multicast_address"].as<std::string>();
        sensor_multicast_port = config["sensor_multicast_port"].as<uint16_t>();
        sensor_multicast_interface = config["sensor_multicast_interface"].as<std::string>();

        time_sync_period_ms = config["time_sync_period_ms"].as<uint32_t>();
        time_sync_address = config["time_sync_address"].as<std::string>();
        time_sync_port = config["time_sync_port"].as<uint16_t>();

        accelerometer_calibration = config["accelerometer_calibration"].as<bool>();
        accelerometer_R = config["accelerometer_R"].as<Eigen::Matrix3f>();
        accelerometer_bias = config["accelerometer_bias"].as<Eigen::Vector3f>();

        position_process_noise = config["position_process_noise"].as<float>();
        velocity_process_noise = config["velocity_process_noise"].as<float>();
        quaterion_process_noise = config["quaterion_process_noise"].as<float>();
        gyro_bias_process_noise = config["gyro_bias_process_noise"].as<float>();
        drawbar_process_noise = config["drawbar_process_noise"].as<float>();
        accel_measurement_noise = config["accel_measurement_noise"].as<float>();
        pos_provider_measurement_noise = config["pos_provider_measurement_noise"].as<float>();
        constraint_correction_scaler = config["constraint_correction_scaler"].as<float>();
        constraint_correction_repeats = config["constraint_correction_repeats"].as<int>();
        drawbar_length = config["drawbar_length"].as<float>();

        loop_rate_ms = config["loop_rate_ms"].as<uint32_t>();
        debug_mode = config["debug_mode"].as<bool>();
    }
    catch(const std::exception& e)
    {
        std::cerr << "Loading default file. Reason: " << e.what() << std::endl;
        restore_defaults();
    }
}

Config::~Config() 
{
    
}

void Config::save() 
{
    try
    {
        YAML::Emitter out;
        out << YAML::BeginMap;

        out << YAML::Comment("Logger config:");
        out << YAML::Key << "log_mask" << YAML::Value << log_mask;
        out << YAML::Key << "log_path" << YAML::Value << log_path;
        out << YAML::Key << "run_counter_path" << YAML::Value << run_counter_path;
        out << YAML::Key << "logger_address" << YAML::Value << logger_address;

        out << YAML::Newline << YAML::Newline;
        out << YAML::Comment("Time sync:");
        out << YAML::Key << "time_sync_period_ms" << YAML::Value << time_sync_period_ms;
        out << YAML::Key << "time_sync_address" << YAML::Value << time_sync_address;
        out << YAML::Key << "time_sync_port" << YAML::Value << time_sync_port;

        out << YAML::Newline << YAML::Newline;
        out << YAML::Comment("Sensors config:");
        out << YAML::Key << "sensor_multicast_address" << YAML::Value << sensor_multicast_address;
        out << YAML::Key << "sensor_multicast_port" << YAML::Value << sensor_multicast_port;
        out << YAML::Key << "sensor_multicast_interface" << YAML::Value << sensor_multicast_interface;

        out << YAML::Key << "accelerometer_calibration" << YAML::Value << accelerometer_calibration;
        out << YAML::Key << "accelerometer_R" << YAML::Value << YAML::convert<Eigen::Matrix3f>::encode(accelerometer_R);
        out << YAML::Key << "accelerometer_bias" << YAML::Value << YAML::convert<Eigen::Vector3f>::encode(accelerometer_bias);

        out << YAML::Newline << YAML::Newline;
        out << YAML::Comment("EKF parameters:");
        out << YAML::Key << "position_process_noise" << YAML::Value << position_process_noise;
        out << YAML::Key << "velocity_process_noise" << YAML::Value << velocity_process_noise;
        out << YAML::Key << "quaterion_process_noise" << YAML::Value << quaterion_process_noise;
        out << YAML::Key << "gyro_bias_process_noise" << YAML::Value << gyro_bias_process_noise;
        out << YAML::Key << "drawbar_process_noise" << YAML::Value << drawbar_process_noise;
        out << YAML::Key << "accel_measurement_noise" << YAML::Value << accel_measurement_noise;
        out << YAML::Key << "pos_provider_measurement_noise" << YAML::Value << pos_provider_measurement_noise;
        out << YAML::Key << "constraint_correction_scaler" << YAML::Value << constraint_correction_scaler;
        out << YAML::Key << "constraint_correction_repeats" << YAML::Value << constraint_correction_repeats;
        out << YAML::Key << "drawbar_length" << YAML::Value << drawbar_length;
        
        out << YAML::Newline << YAML::Newline;
        out << YAML::Comment("Other:");
        out << YAML::Key << "loop_rate_ms" << YAML::Value << loop_rate_ms;
        out << YAML::Key << "debug_mode" << YAML::Value << debug_mode;
        out << YAML::EndMap;
        std::ofstream fout(config_file_path, std::ios::out | std::ios::trunc);
        fout << out.c_str();
        fout.close();
        Logger(LogType::INFO)("Config saved!");
    }
    catch(const std::exception& e)
    {
        Logger(LogType::ERROR)(std::string("Config could not be saved. Reason: ") + e.what()); 
    }
}

const Config &Config::get_singleton()
{
    if(singleton == nullptr)
    {
        singleton = std::make_unique<Config>(config_file_path);
    }
    return *singleton;
}

Config &Config::get_singleton_mut() {
    if(singleton == nullptr)
    {
        singleton = std::make_unique<Config>(config_file_path);
    }
    return *singleton;
}

void Config::restore_defaults() 
{
    // Logger config:
    log_mask = -1;
    log_path = "../logs/";
    run_counter_path = "../logs/run_counter";
    logger_address = "tcp://*:6666";

    // Time sync:
    time_sync_period_ms = 5000;
    time_sync_address = "224.0.0.100";
    time_sync_port = 50000;

    // Sensors config:
    sensor_multicast_address = "224.0.0.100";
    sensor_multicast_port = 1234;
    sensor_multicast_interface = "";
    accelerometer_calibration = true;
    accelerometer_R = Eigen::Matrix3f::Identity();
    accelerometer_bias = Eigen::Vector3f::Zero();

    // EKF parameters:
    position_process_noise = 0.01f;
    velocity_process_noise = 0.01f;
    quaterion_process_noise = 0.01f;
    gyro_bias_process_noise = 0.01f;
    drawbar_process_noise = 0.01f;
    accel_measurement_noise = 0.01f;
    pos_provider_measurement_noise = 0.01f;
    constraint_correction_scaler = 0.01f;
    constraint_correction_repeats = 3;
    drawbar_length = 0.0f;

    // Other:
    loop_rate_ms = 5;
    debug_mode = false;

    save();
}
