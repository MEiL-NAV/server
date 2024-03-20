#pragma once
#include "EKFConstraints.h"
#include "ConstraintsLoader.h"
#include "Math.h"
#include <optional>


//State: x, y, z, vx, vy, vz, q0, qx, qy, qz, gyro_bias_x, gyro_bias_y, gyro_bias_z, drawbar_az, drawbar_el
class EKF_IMU : private EKFConstraints<15, 6, 6>
{
public:
    EKF_IMU(float drawbar_length = 0.0f);
    virtual ~EKF_IMU() {}

    void update(uint32_t reading_time,
                Eigen::Vector3f gyro_reading,
                Eigen::Vector3f acc_reading,
                std::optional<Eigen::Vector3f> pos_provider_reading = std::nullopt);

    Eigen::Vector<float, 15> get_state() { return EKF::get_state(); }
    Eigen::Vector3f get_position() { return state.head<3>(); }
    Eigen::Vector3f get_velocity() { return state.segment<3>(3); }
    Eigen::Vector4f get_quaterion() { return state.segment<4>(6); }
    Eigen::Vector3f get_rpy() { return quaterion_to_rpy(get_quaterion()); }

    void reset();

    // EKF parameters
    void set_position_process_noise(float position_process_noise);
    void set_velocity_process_noise(float velocity_process_noise);
    void set_quaterion_process_noise(float quaterion_process_noise);
    void set_gyro_bias_process_noise(float gyro_bias_process_noise);
    void set_drawbar_process_noise(float drawbar_process_noise);


    void set_accel_measurement_noise(float accel_measurement_noise);
    void set_pos_provider_measurement_noise(float pos_provider_measurement_noise);

    void set_constraint_correction_scaler(float constraint_correction_scaler);
    void set_constraint_correction_repeats(int constraint_correction_repeats);

protected:
    uint32_t last_update;
    float delta_time;
    ConstraintsLoader constraints_loader;
    const float drawbar_length;

    Eigen::Vector<float, 15>
    state_function(Eigen::Vector<float, 15> &state,
                   Eigen::Vector<float, 6> input) override;
    
    Eigen::Matrix<float,15,15>
    state_function_derivative(Eigen::Vector<float, 15> &state,
                   Eigen::Vector<float, 6> input) override;

    Eigen::Vector<float, 6>
    measurement_function(Eigen::Vector<float, 15> &state) override;

    Eigen::Matrix<float,6,15>
    measurement_function_derivative(Eigen::Vector<float, 15> &state) override;

    Eigen::VectorXf
    constraints(const Eigen::Vector<float, 15> &state) override;

    Eigen::MatrixXf
    constraints_derivative(const Eigen::Vector<float, 15> &state) override;

    void wrap_angles();
                   
    Eigen::Vector<float, 3> g;
    static constexpr float max_delta_time = 0.1f;
};