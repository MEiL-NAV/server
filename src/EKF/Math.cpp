#include "Math.h"

// Convert a quaternion to a direction cosine matrix. Quaterion is given as q0 qx qy qz
Eigen::Matrix3f quaterion_to_dcm(Eigen::Vector4f& quaterion)
{
   Eigen::Matrix3f dcm;
   float& q0 = quaterion(0);
   float& qx = quaterion(1);
   float& qy = quaterion(2);
   float& qz = quaterion(3);

   dcm(0, 0) = q0 * q0 + qx * qx - qy * qy - qz * qz;
   dcm(1, 0) = 2 * (qx * qy - q0 * qz);
   dcm(2, 0) = 2 * (qx * qz + q0 * qy);
   dcm(0, 1) = 2 * (qx * qy + q0 * qz);
   dcm(1, 1) = q0 * q0 - qx * qx + qy * qy - qz * qz;
   dcm(2, 1) = 2 * (qy * qz - q0 * qx);
   dcm(0, 2) = 2 * (qx * qz - q0 * qy);
   dcm(1, 2) = 2 * (qy * qz + q0 * qx);
   dcm(2, 2) = q0 * q0 - qx * qx - qy * qy + qz * qz;

   return dcm;
}

// Convert a quaternion to roll pitch yaw. Quaterion is given as q0 qx qy qz
Eigen::Vector3f quaterion_to_rpy(Eigen::Vector4f quaterion)
{
    Eigen::Vector3f rpy;
    float& q0 = quaterion(0);
    float& qx = quaterion(1);
    float& qy = quaterion(2);
    float& qz = quaterion(3);

    rpy(0) = atan2(2 * (q0 * qx + qy * qz), q0 * q0 - qx * qx - qy * qy + qz * qz);
    rpy(1) = asin(2 * (q0 * qy - qx * qz));
    rpy(2) = atan2(2 * (q0 * qz + qx * qy), q0 * q0 - qx * qx - qy * qy - qz * qz);

    return rpy;
}

Eigen::Matrix<float,4,3> S(Eigen::Vector4f& quaterion)
{
    Eigen::Matrix<float,4,3> S;
    S(0,0) = -quaterion(1);
    S(0,1) = -quaterion(2);
    S(0,2) = -quaterion(3);
    S(1,0) = quaterion(0);
    S(1,1) = -quaterion(3);
    S(1,2) = quaterion(2);
    S(2,0) = quaterion(3);
    S(2,1) = quaterion(0);
    S(2,2) = -quaterion(1);
    S(3,0) = -quaterion(2);
    S(3,1) = quaterion(1);
    S(3,2) = quaterion(0);
    return S;
}

Eigen::Matrix<float,3,4> Ca(Eigen::Vector4f& quaterion)
{
    Eigen::Matrix<float,3,4> Ca;
    Ca(0,0) = 2 * quaterion(2);
    Ca(0,1) = 2 * quaterion(3);
    Ca(0,2) = 2 * quaterion(0);
    Ca(0,3) = 2 * quaterion(1);
    Ca(1,0) = -2 * quaterion(1);
    Ca(1,1) = -2 * quaterion(0);
    Ca(1,2) = 2 * quaterion(3);
    Ca(1,3) = 2 * quaterion(2);
    Ca(2,0) = 2 * quaterion(0);
    Ca(2,1) = -2 * quaterion(1);
    Ca(2,2) = -2 * quaterion(2);
    Ca(2,3) = 2 * quaterion(3);
    return Ca;
}

Eigen::Matrix<float,3,4> D(Eigen::Vector4f& quaterion, Eigen::Vector3f& acc)
{
    Eigen::Matrix<float,3,4> D;

    float& q0 = quaterion(0);
    float& qx = quaterion(1);
    float& qy = quaterion(2);
    float& qz = quaterion(3);

    float& ax = acc(0);
    float& ay = acc(1);
    float& az = acc(2);

    D(0,0) = ax*q0 + ay*qz - az*qy;
    D(1,0) = ay*q0 - ax*qz + az*qx;
    D(2,0) = az*q0 + ax*qy - ay*qx;

    D(0,1) = ax*qx + ay*qy + az*qz;
    D(1,1) = az*q0 + ax*qy - ay*qx;
    D(2,1) = ax*qz - ax*qz - az*qx;

    D(0,2) = ay*qx - ax*qy - az*q0;
    D(1,2) = ax*qx + ay*qy + az*qz;
    D(2,2) = ax*q0 - ay*qz - az*qy;

    D(0,3) = ay*q0 - ax*qz + az*qx;
    D(1,3) = az*qy - ay*qz - ax*q0;
    D(2,3) = ax*qx + ay*qy + az*qz;
    
    return 2.0f*D;
}