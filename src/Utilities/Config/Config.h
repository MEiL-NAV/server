#pragma once

#include <cstdint>
#include <memory>

class Config
{
public:
    Config(const char* config_file_path);
    ~Config();

    int log_mask;
    std::string log_path;
    std::string run_counter_path;
    std::string logger_address;

    uint32_t time_sync_period_ms;
    uint16_t time_sync_port;

    bool accelerometer_calibration;
    float accelerometer_bias_x;
    float accelerometer_bias_y;
    float accelerometer_bias_z;
    float accelerometer_scalers_x;
    float accelerometer_scalers_y;
    float accelerometer_scalers_z;

    uint32_t loop_rate_ms;

    static const Config& get_singleton();

    static Config& get_singleton_mut();

private:
    void restore_defaults();

    static std::unique_ptr<Config> singleton;

    constexpr static const char* config_file_path = "config.yaml";
};