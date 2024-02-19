#include "EKF.h"

template<int state_size, int input_size, int measurement_size>
class EKFConstraints : public EKF<state_size, input_size, measurement_size>
{
public:
    virtual ~EKFConstraints() {}

    Eigen::Vector<float, state_size> apply_constraints(Eigen::Vector<float, state_size> state, Eigen::Matrix<float, state_size, state_size> correction);

protected:
    float constraint_correction_scaler = 1.0f;

    virtual Eigen::Vector<float, state_size>
    constraints(const Eigen::Vector<float, state_size> &state) = 0;

    virtual Eigen::Matrix<float, state_size, state_size>
    constraints_derivative(const Eigen::Vector<float, state_size> &state) = 0;
};

template <int state_size, int input_size, int measurement_size>
inline Eigen::Vector<float, state_size> EKFConstraints<state_size, input_size, measurement_size>::apply_constraints(
    Eigen::Vector<float, state_size> state, Eigen::Matrix<float, state_size, state_size> correction)
{
    Eigen::Matrix<float, state_size, state_size> D = constraints_derivative(state);
    Eigen::Vector<float, state_size> d = -constraints(state) + D * state;

    Eigen::Matrix<float, state_size, state_size> P_inv = (D * (correction * D.transpose())).completeOrthogonalDecomposition().pseudoInverse();

    return state - constraint_correction_scaler * correction * D.transpose() * P_inv * (D * state - d);
}
