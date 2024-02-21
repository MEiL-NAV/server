#include "EKF_IMU.h"
#include <iostream>


EKF_IMU::EKF_IMU()
    :   EKFConstraints(1e-6f * Eigen::Matrix<float, 13, 13>::Identity(),
                       1e-10f *Eigen::Matrix<float, 3, 3>::Identity()),
        last_update{0},
        delta_time{0.0f},
        g{0.0f,0.0f,9.805f}
{
    state(3+3) = 1.0f; // init quaterion as 1,0,0,0

    // TODO: calibrate
    constraint_correction_scaler = 5e-2f;
}

void EKF_IMU::update(uint32_t reading_time, Eigen::Vector3f gyro_reading,
                     Eigen::Vector3f acc_reading) 
{
    delta_time = (reading_time - last_update) / 1e3f;
    last_update = reading_time;
    if (delta_time < max_delta_time)
    {
        Eigen::Vector<float, 6> input;
        input.head<3>() = gyro_reading;
        input.tail<3>() = acc_reading;
        predict(input);
        correct(acc_reading);
        apply_constraints(state,covariance);
    }
}

Eigen::Vector<float, 13> EKF_IMU::state_function(Eigen::Vector<float, 13> &state, Eigen::Vector<float, 6> input)
{
    Eigen::Vector<float, 13> new_state = state;
    Eigen::Vector<float, 4> quaterion = state.segment<4>(6);
    Eigen::Vector<float, 3> acc = quaterion_to_dcm(quaterion) * input.tail<3>() - g;
    new_state.head<3>() += delta_time * state.segment<3>(3) 
        + 0.5f * delta_time * delta_time * acc;
    new_state.segment<3>(3) +=  delta_time * acc;
    new_state.segment<4>(6) += 0.5f * delta_time * S(quaterion) * (input.head<3>() - state.tail<3>());
    return new_state;
}

Eigen::Matrix<float, 13, 13> EKF_IMU::state_function_derivative(Eigen::Vector<float, 13> &state, Eigen::Vector<float, 6> input)
{
    Eigen::Matrix<float, 13, 13> A = Eigen::Matrix<float, 13, 13>::Identity();
    A.block<3, 3>(0, 3) = Eigen::Matrix<float, 3, 3>::Identity() * delta_time;
    Eigen::Vector<float, 4> quaterion = state.segment<4>(6);
    Eigen::Vector<float, 3> acc_val = input.tail<3>();
    Eigen::Matrix<float,3,4> D_val = D(quaterion, acc_val);
    A.block<3, 4>(0,6) = delta_time * delta_time / 2.0f * D_val;
    A.block<3, 4>(3,6) = delta_time * D_val;
    A.block<4, 3>(6,10) = - delta_time / 2.0f * S(quaterion);
    return A;
}

Eigen::Vector<float, 3> EKF_IMU::measurement_function(Eigen::Vector<float, 13> &state)
{
    Eigen::Vector3f h;

    float& q0 = state(6);
    float& qx = state(7);
    float& qy = state(8);
    float& qz = state(9);

    h(0) = 2 * q0 * qy + 2 * qx * qz;
    h(1) = 2 * qy * qz - 2 * q0 * qx;
    h(2) = q0 * q0 - qx * qx - qy * qy + qz * qz;
    return g.z() * h;
}

Eigen::Matrix<float, 3, 13> EKF_IMU::measurement_function_derivative(Eigen::Vector<float, 13> &state)
{
    Eigen::Matrix<float, 3, 13> H = Eigen::Matrix<float, 3, 13>::Zero();
    Eigen::Vector<float, 4> quaterion = state.segment<4>(6);
    H.block<3, 4>(0, 6) = g.z() * Ca(quaterion);
    return H;
}

Eigen::VectorXf EKF_IMU::constraints(const Eigen::Vector<float, 13> &state)
{
    if(constraints_loader.is_valid())
    {
        return constraints_loader.constraints(state);
    }
    return Eigen::VectorXf::Zero(1);
}

Eigen::MatrixXf EKF_IMU::constraints_derivative(const Eigen::Vector<float, 13> &state)
{
    if(constraints_loader.is_valid())
    {
        return constraints_loader.constraints_derivative(state);
    }
    return Eigen::MatrixXf::Identity(1,13);
}
