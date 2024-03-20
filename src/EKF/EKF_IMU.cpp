#include "EKF_IMU.h"
#include <iostream>
#include "../Utilities/Loggers/Logger.h"


EKF_IMU::EKF_IMU(float drawbar_length)
    :   EKFConstraints(),
        last_update{0},
        delta_time{0.0f},
        drawbar_length{drawbar_length},
        g{0.0f,0.0f,9.805f}
{
    reset();
}

void EKF_IMU::update(uint32_t reading_time, Eigen::Vector3f gyro_reading,
                     Eigen::Vector3f acc_reading, std::optional<Eigen::Vector3f> pos_provider_reading) 
{
    delta_time = (reading_time - last_update) / 1e3f;
    last_update = reading_time;
    if (delta_time < max_delta_time)
    {
        Eigen::Vector<float, 6> input;
        input.head<3>() = gyro_reading;
        input.tail<3>() = acc_reading;
        predict(input);
        if (pos_provider_reading.has_value())
        {
            Eigen::Vector<float, 6> acc_and_pos;
            acc_and_pos.head<3>() = acc_reading;
            acc_and_pos.tail<3>() = pos_provider_reading.value();
            correct(acc_and_pos);
        }
        else
        {
            correct(acc_reading);
        }
        state = apply_constraints(state,covariance);
        wrap_angles();
    }
    if (state.hasNaN())
    {
        reset();
        Logger(LogType::ERROR)("EKF IMU reseted due to NaN state");
    }
}

Eigen::Vector<float, 15> EKF_IMU::state_function(Eigen::Vector<float, 15> &state, Eigen::Vector<float, 6> input)
{
    Eigen::Vector<float, 15> new_state = state;
    Eigen::Vector<float, 4> quaterion = state.segment<4>(6);
    Eigen::Vector<float, 3> acc = quaterion_to_dcm(quaterion) * input.tail<3>() - g;
    new_state.head<3>() += delta_time * state.segment<3>(3) 
        + 0.5f * delta_time * delta_time * acc;
    new_state.segment<3>(3) +=  delta_time * acc;
    new_state.segment<4>(6) += 0.5f * delta_time * S(quaterion) * (input.head<3>() - state.tail<3>());
    if (drawbar_length <= 0.001f)
    {
        new_state(13) = 0.0f;
        new_state(14) = 0.0f;
    }
    return new_state;
}

Eigen::Matrix<float, 15, 15> EKF_IMU::state_function_derivative(Eigen::Vector<float, 15> &state, Eigen::Vector<float, 6> input)
{
    Eigen::Matrix<float, 15, 15> A = Eigen::Matrix<float, 15, 15>::Identity();
    A.block<3, 3>(0, 3) = Eigen::Matrix<float, 3, 3>::Identity() * delta_time;
    Eigen::Vector<float, 4> quaterion = state.segment<4>(6);
    Eigen::Vector<float, 3> acc_val = input.tail<3>();
    Eigen::Matrix<float,3,4> D_val = D(quaterion, acc_val);
    A.block<3, 4>(0,6) = delta_time * delta_time / 2.0f * D_val;
    A.block<3, 4>(3,6) = delta_time * D_val;
    A.block<4, 3>(6,10) = - delta_time / 2.0f * S(quaterion);
    return A;
}

Eigen::Vector<float, 6> EKF_IMU::measurement_function(Eigen::Vector<float, 15> &state)
{
    Eigen::Vector<float,6> h;

    float& q0 = state(6);
    float& qx = state(7);
    float& qy = state(8);
    float& qz = state(9);

    float& az = state(13);
    float& el = state(14);

    h(0) = (2 * q0 * qy + 2 * qx * qz) * g.z();
    h(1) = (2 * qy * qz - 2 * q0 * qx) * g.z();
    h(2) = (q0 * q0 - qx * qx - qy * qy + qz * qz) * g.z();
    h.tail<3>() = state.head<3>();

    if (drawbar_length > 0.001f)
    {
        float cos_el = cosf(el);
        h(3) += drawbar_length * cosf(az) * cos_el;
        h(4) += drawbar_length * sinf(az) * cos_el;
        h(5) += drawbar_length * sinf(el);
    }

    return h;
}

Eigen::Matrix<float, 6, 15> EKF_IMU::measurement_function_derivative(Eigen::Vector<float, 15> &state)
{
    Eigen::Matrix<float, 6, 15> H = Eigen::Matrix<float, 6, 15>::Zero();
    Eigen::Vector<float, 4> quaterion = state.segment<4>(6);
    H.block<3, 4>(0, 6) = g.z() * Ca(quaterion);
    H.block<3, 3>(3, 0) = Eigen::Matrix<float, 3, 3>::Identity();
    if (drawbar_length > 0.001f)
    {
        float& az = state(13);
        float& el = state(14);
        float cos_el = cosf(el);
        float sin_el = sinf(el);
        float cos_az = cosf(az);
        float sin_az = sinf(az);
        H(3,13) = -drawbar_length * sin_az * cos_el;
        H(3,14) = -drawbar_length * cos_az * sin_el;
        H(4,13) =  drawbar_length * cos_az * cos_el;
        H(4,14) = -drawbar_length * sin_az * sin_el;
        H(5,14) =  drawbar_length * cos_el;
    }
    return H;
}

Eigen::VectorXf EKF_IMU::constraints(const Eigen::Vector<float, 15> &state)
{
    if(constraints_loader.is_valid())
    {
        return constraints_loader.constraints(state);
    }
    return Eigen::VectorXf::Zero(1);
}

Eigen::MatrixXf EKF_IMU::constraints_derivative(const Eigen::Vector<float, 15> &state)
{
    if(constraints_loader.is_valid())
    {
        return constraints_loader.constraints_derivative(state);
    }
    return Eigen::MatrixXf::Ones(1,15);
}

void EKF_IMU::wrap_angles() 
{
    state(13) = wrap_to_pi(state(13));
    state(14) = wrap_to_pi(state(14));
}

void EKF_IMU::reset()
{
    state.setZero();
    state(3+3) = 1.0f; // init quaterion as 1,0,0,0
    covariance = process_noise_covariance;
}

void EKF_IMU::set_position_process_noise(float position_process_noise)
{
    process_noise_covariance.block<3,3>(0,0) = Eigen::Matrix<float,3,3>::Identity() * position_process_noise;
    covariance = process_noise_covariance;
}

void EKF_IMU::set_velocity_process_noise(float velocity_process_noise) 
{
    process_noise_covariance.block<3,3>(3,3) = Eigen::Matrix<float,3,3>::Identity() * velocity_process_noise;
    covariance = process_noise_covariance;
}

void EKF_IMU::set_quaterion_process_noise(float quaterion_process_noise) 
{
    process_noise_covariance.block<4,4>(6,6) = Eigen::Matrix<float,4,4>::Identity() * quaterion_process_noise;
    covariance = process_noise_covariance;
}

void EKF_IMU::set_gyro_bias_process_noise(float gyro_bias_process_noise) 
{
    process_noise_covariance.block<3,3>(10,10) = Eigen::Matrix<float,3,3>::Identity() * gyro_bias_process_noise;
    covariance = process_noise_covariance;
}

void EKF_IMU::set_drawbar_process_noise(float drawbar_process_noise) 
{
    process_noise_covariance.block<2,2>(13,13) = Eigen::Matrix<float,2,2>::Identity() * drawbar_process_noise;
    covariance = process_noise_covariance;
}

void EKF_IMU::set_accel_measurement_noise(float accel_measurement_noise) 
{
    measurement_noise_covariance.block<3,3>(0,0) = Eigen::Matrix<float,3,3>::Identity() * accel_measurement_noise;
}

void EKF_IMU::set_pos_provider_measurement_noise(
    float pos_provider_measurement_noise) 
{
    measurement_noise_covariance.block<3,3>(3,3) = Eigen::Matrix<float,3,3>::Identity() * pos_provider_measurement_noise;
}

void EKF_IMU::set_constraint_correction_scaler(
    float constraint_correction_scaler) 
{
    this->constraint_correction_scaler = constraint_correction_scaler;
}

void EKF_IMU::set_constraint_correction_repeats(
    int constraint_correction_repeats) 
{
    this->constraint_correction_repeats = constraint_correction_repeats;
}
