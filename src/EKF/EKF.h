#pragma once
#include <eigen3/Eigen/Dense>

template<int state_size, int input_size, int measurement_size>
class EKF
{
public:
    EKF(
        Eigen::Matrix<float,state_size,state_size> process_noise_covariance,
        Eigen::Matrix<float,measurement_size,measurement_size> measurement_noise_covariance
    );
    virtual ~EKF() {}

    virtual void predict(Eigen::Vector<float, input_size> input);
    virtual void correct(Eigen::Vector<float, measurement_size> measurement);

    Eigen::Vector<float, state_size> get_state() { return state; }

protected:
    Eigen::Vector<float,state_size> state;
    Eigen::Vector<float,state_size> predicted_state;

    Eigen::Matrix<float,state_size,state_size> covariance;
    Eigen::Matrix<float,state_size,state_size> predicted_covariance;

    Eigen::Matrix<float,state_size,state_size> process_noise_covariance;
    Eigen::Matrix<float,measurement_size,measurement_size> measurement_noise_covariance;

    virtual Eigen::Vector<float, state_size>
    state_function(Eigen::Vector<float, state_size> &state,
                   Eigen::Vector<float, input_size> input) = 0;
    
    virtual Eigen::Matrix<float,state_size,state_size>
    state_function_derivative(Eigen::Vector<float, state_size> &state,
                   Eigen::Vector<float, input_size> input) = 0;

    virtual Eigen::Vector<float, measurement_size>
    measurement_function(Eigen::Vector<float, state_size> &state) = 0;

    virtual Eigen::Matrix<float,measurement_size,state_size>
    measurement_function_derivative(Eigen::Vector<float, state_size> &state) = 0;
};

template <int state_size, int input_size, int measurement_size>
inline EKF<state_size, input_size, measurement_size>::EKF(
    Eigen::Matrix<float,state_size,state_size> process_noise_covariance,
    Eigen::Matrix<float,measurement_size,measurement_size> measurement_noise_covariance)
    : process_noise_covariance{process_noise_covariance},
      measurement_noise_covariance{measurement_noise_covariance}
{
    state.setZero();
    covariance = process_noise_covariance;
}

template <int state_size, int input_size, int measurement_size>
inline void EKF<state_size, input_size, measurement_size>::predict(Eigen::Vector<float, input_size> input)
{
    predicted_state = state_function(state,input);
    Eigen::Matrix<float,state_size,state_size> A = state_function_derivative(state, input);
    predicted_covariance = A * covariance * A.transpose() + process_noise_covariance;
}

template <int state_size, int input_size, int measurement_size>
inline void EKF<state_size, input_size, measurement_size>::correct(Eigen::Vector<float, measurement_size> measurement)
{
    Eigen::Matrix<float,measurement_size,state_size> H = measurement_function_derivative(predicted_state);
    Eigen::Matrix<float,state_size,measurement_size> K = predicted_covariance * H.transpose() 
        * (H * predicted_covariance * H.transpose() + measurement_noise_covariance).inverse();
    state = predicted_state + K * (measurement - measurement_function(predicted_state));
    covariance = (Eigen::Matrix<float,state_size,state_size>::Identity() - K * H) * predicted_covariance;
}
