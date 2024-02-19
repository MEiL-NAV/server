#include "EKFConstraints.h"


//State: x, y, z, vx, vy, vz, q0, qx, qy, qz, gyro_bias_x, gyro_bias_y, gyro_bias_z 

class EKF_IMU : private EKFConstraints<13, 6, 6>
{
public:
    EKF_IMU();
    virtual ~EKF_IMU() {}

    Eigen::Vector3f get_position() { return state.head<3>(); }
    Eigen::Vector3f get_velocity() { return state.segment<3>(3); }
    Eigen::Vector4f get_quaterion() { return state.segment<4>(6); }

protected:
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

    Eigen::Vector<float, 13>
    constraints(const Eigen::Vector<float, 13> &state) override;

    Eigen::Matrix<float, 13, 13>
    constraints_derivative(const Eigen::Vector<float, 13> &state) override;
};