#pragma once

#include <eigen3/Eigen/Dense>
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

    std::string sensor_multicast_address;
    uint16_t sensor_multicast_port;
    std::string sensor_multicast_interface;

    uint32_t time_sync_period_ms;
    std::string time_sync_address;
    uint16_t time_sync_port;

    bool accelerometer_calibration;
    Eigen::Matrix3f accelerometer_R;
    Eigen::Vector3f accelerometer_bias;

    float position_process_noise;
    float velocity_process_noise;
    float quaterion_process_noise;
    float gyro_bias_process_noise;
    float accel_measurement_noise;
    float pos_provider_measurement_noise;
    float constraint_correction_scaler;

    uint32_t loop_rate_ms;

    static const Config& get_singleton();

    static Config& get_singleton_mut();

private:
    void restore_defaults();

    static std::unique_ptr<Config> singleton;

    constexpr static const char* config_file_path = "config.yaml";
};