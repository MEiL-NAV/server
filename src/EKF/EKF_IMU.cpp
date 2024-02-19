#include "EKF_IMU.h"

EKF_IMU::EKF_IMU()
{

    // TODO: calibrate
    constraint_correction_scaler = 0.0f;
}

Eigen::Vector<float, 13> EKF_IMU::state_function(Eigen::Vector<float, 13> &state, Eigen::Vector<float, 6> input)
{
    return Eigen::Vector<float, 13>();
}

Eigen::Matrix<float, 13, 13> EKF_IMU::state_function_derivative(Eigen::Vector<float, 13> &state, Eigen::Vector<float, 6> input)
{
    return Eigen::Matrix<float, 13, 13>();
}

Eigen::Vector<float, 6> EKF_IMU::measurement_function(Eigen::Vector<float, 13> &state)
{
    return Eigen::Vector<float, 6>();
}

Eigen::Matrix<float, 6, 13> EKF_IMU::measurement_function_derivative(Eigen::Vector<float, 13> &state)
{
    return Eigen::Matrix<float, 6, 13>();
}

Eigen::Vector<float, 13> EKF_IMU::constraints(const Eigen::Vector<float, 13> &state)
{
    // TODO: implement
    return Eigen::Vector<float, 13>::Zero();
}

Eigen::Matrix<float, 13, 13> EKF_IMU::constraints_derivative(const Eigen::Vector<float, 13> &state)
{
    // TODO: implement
    return Eigen::Matrix<float, 13, 13>::Identity();
}
