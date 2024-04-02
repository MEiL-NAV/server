#pragma once
#include <eigen3/Eigen/Dense>

Eigen::Matrix3f quaterion_to_dcm(Eigen::Vector4f& quaterion);
Eigen::Vector3f quaterion_to_rpy(Eigen::Vector4f quaterion);
Eigen::Matrix<float,4,3> S(Eigen::Vector4f& quaterion);
Eigen::Matrix<float,3,4> Ca(Eigen::Vector4f& quaterion);
Eigen::Matrix<float,3,4> D(Eigen::Vector4f& quaterion, Eigen::Vector3f& acc);
float wrap_to_pi(float angle);
