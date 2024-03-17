#pragma once
#include "EKF.h"
#include "../Utilities/Loggers/Logger.h"

template<int state_size, int input_size, int measurement_size>
class EKFConstraints : public EKF<state_size, input_size, measurement_size>
{
public:
    EKFConstraints() = default;
    virtual ~EKFConstraints() {}

    Eigen::Vector<float, state_size> apply_constraints(Eigen::Vector<float, state_size> state, Eigen::Matrix<float, state_size, state_size> covariance);

protected:
    float constraint_correction_scaler = 1.0f;

    virtual Eigen::VectorXf
    constraints(const Eigen::Vector<float, state_size> &state) = 0;

    virtual Eigen::MatrixXf
    constraints_derivative(const Eigen::Vector<float, state_size> &state) = 0;
};

template <int state_size, int input_size, int measurement_size>
inline Eigen::Vector<float, state_size> EKFConstraints<state_size, input_size, measurement_size>::apply_constraints(
    Eigen::Vector<float, state_size> state, Eigen::Matrix<float, state_size, state_size> covariance)
{
    Eigen::MatrixXf D = constraints_derivative(state);
    Eigen::VectorXf d = -constraints(state) + D * state;

    // correction without covariance
    // Eigen::MatrixXf P_inv = (D * (D.transpose())).completeOrthogonalDecomposition().pseudoInverse();
    // Eigen::Vector<float, state_size> correction = constraint_correction_scaler * D.transpose() * P_inv * (D * state - d);

    Eigen::MatrixXf P_inv = (D * (covariance * D.transpose())).completeOrthogonalDecomposition().pseudoInverse();
    Eigen::Vector<float, state_size> correction = constraint_correction_scaler * covariance * D.transpose() * P_inv * (D * state - d);

    if(correction.hasNaN())
    {
        Logger(LogType::ERROR)("Constraints skiped due to NaN");
        return state;
    }
    return state - correction;
}
