#include "Config.h"
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "../Loggers/Logger.h"

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

        time_sync_period_ms = config["time_sync_period_ms"].as<uint32_t>();
        time_sync_port = config["time_sync_port"].as<uint16_t>();

        accelerometer_calibration = config["accelerometer_calibration"].as<bool>();
        accelerometer_bias_x = config["accelerometer_bias_x"].as<float>();
        accelerometer_bias_y = config["accelerometer_bias_y"].as<float>();
        accelerometer_bias_z = config["accelerometer_bias_z"].as<float>();
        accelerometer_scalers_x = config["accelerometer_scalers_x"].as<float>();
        accelerometer_scalers_y = config["accelerometer_scalers_y"].as<float>();
        accelerometer_scalers_z = config["accelerometer_scalers_z"].as<float>();

        loop_rate_ms = config["loop_rate_ms"].as<uint32_t>();
    }
    catch(const std::exception& e)
    {
        std::cerr << "Loading default file. Reason: " << e.what() << std::endl;
        restore_defaults();
    }
}

Config::~Config() 
{
    try
    {
        std::ofstream fout(config_file_path, std::ios::out | std::ios::trunc);
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
        out << YAML::Key << "time_sync_port" << YAML::Value << time_sync_port;

        out << YAML::Newline << YAML::Newline;
        out << YAML::Comment("Sensors config:");
        out << YAML::Key << "accelerometer_calibration" << YAML::Value << accelerometer_calibration;
        out << YAML::Key << "accelerometer_bias_x" << YAML::Value << accelerometer_bias_x;
        out << YAML::Key << "accelerometer_bias_y" << YAML::Value << accelerometer_bias_y;
        out << YAML::Key << "accelerometer_bias_z" << YAML::Value << accelerometer_bias_z;
        out << YAML::Key << "accelerometer_scalers_x" << YAML::Value << accelerometer_scalers_x;
        out << YAML::Key << "accelerometer_scalers_y" << YAML::Value << accelerometer_scalers_y;
        out << YAML::Key << "accelerometer_scalers_z" << YAML::Value << accelerometer_scalers_z;
        
        out << YAML::Newline << YAML::Newline;
        out << YAML::Comment("Other:");
        out << YAML::Key << "loop_rate_ms" << YAML::Value << loop_rate_ms;
        out << YAML::EndMap;
        fout << out.c_str();
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
    time_sync_port = 50000;

    // Sensors config:
    accelerometer_calibration = true;
    accelerometer_bias_x = 0.0f;
    accelerometer_bias_y = 0.0f;
    accelerometer_bias_z = 0.0f;
    accelerometer_scalers_x = 1.0f;
    accelerometer_scalers_y = 1.0f;
    accelerometer_scalers_z = 1.0f;

    // Other:
    loop_rate_ms = 5;
}
