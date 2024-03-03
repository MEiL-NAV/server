#pragma once
#include "EKFConstraints.h"
#include "ConstraintsLoader.h"
#include "Math.h"


//State: x, y, z, vx, vy, vz, q0, qx, qy, qz, gyro_bias_x, gyro_bias_y, gyro_bias_z 
class EKF_IMU : private EKFConstraints<13, 6, 6>
{
public:
    EKF_IMU();
    virtual ~EKF_IMU() {}

    void update(uint32_t reading_time, Eigen::Vector3f gyro_reading, Eigen::Vector3f acc_reading);

    Eigen::Vector<float, 13> get_state() { return EKF::get_state(); }
    Eigen::Vector3f get_position() { return state.head<3>(); }
    Eigen::Vector3f get_velocity() { return state.segment<3>(3); }
    Eigen::Vector4f get_quaterion() { return state.segment<4>(6); }
    Eigen::Vector3f get_rpy() { return quaterion_to_rpy(get_quaterion()); }

    // EKF parameters
    void set_position_process_noise(float position_process_noise);
    void set_velocity_process_noise(float velocity_process_noise);
    void set_quaterion_process_noise(float quaterion_process_noise);
    void set_gyro_bias_process_noise(float gyro_bias_process_noise);

    void set_accel_measurement_noise(float accel_measurement_noise);
    void set_pos_provider_measurement_noise(float pos_provider_measurement_noise);

    void set_constraint_correction_scaler(float constraint_correction_scaler);

protected:
    uint32_t last_update;
    float delta_time;
    ConstraintsLoader constraints_loader;

    Eigen::Vector<float, 13>
    state_function(Eigen::Vector<float, 13> &state,
                   Eigen::Vector<float, 6> input) override;
    
    Eigen::Matrix<float,13,13>
    state_function_derivative(Eigen::Vector<float, 13> &state,
                   Eigen::Vector<float, 6> input) override;

    Eigen::Vector<float, 6>
    measurement_function(Eigen::Vector<float, 13> &state) override;

    Eigen::Matrix<float,6,13>
    measurement_function_derivative(Eigen::Vector<float, 13> &state) override;

    Eigen::VectorXf
    constraints(const Eigen::Vector<float, 13> &state) override;

    Eigen::MatrixXf
    constraints_derivative(const Eigen::Vector<float, 13> &state) override;
                   
    Eigen::Vector<float, 3> g;
    static constexpr float max_delta_time = 0.1f;
};