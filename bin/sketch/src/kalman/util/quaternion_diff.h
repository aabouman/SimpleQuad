#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/src/kalman/util/quaternion_diff.h"
#ifndef QUAT_MATH_H
#define QUAT_MATH_H

// The Arduino max and min macros must be undef'ed to get Eigen to compile
#undef max
#undef min
#include <ArduinoEigenDense.h>
#include <ArduinoEigen/Eigen/Geometry>

using namespace Eigen;

/********************************************************************
 *                    Rotation Error Functions                      *
 ********************************************************************/
// Compute the error between two quaternions "q2 - q1"
template <typename T>
Vector3<T> rotation_error(Quaternion<T> q2, Quaternion<T> q1)
{
    Quaternion<T> tmp = q1.conjugate() * q2;
    Vector3<T> quat_err = (1.0 / tmp.w()) * tmp.vec();
    return quat_err;
}

// Add a quaternion with a quaternion differential
template <typename T>
Quaternion<T> add_rotation_error(Quaternion<T> q, Vector3<T> q_err)
{
    float m = 1 / sqrt(1 + q_err.transpose() * q_err);
    Quaternion<T> tmp(m, m * q_err(0), m * q_err(1), m * q_err(2));
    return q * tmp;
}

// Differential map
template <typename T>
Matrix<T, 4, 3> differential_quat(Quaternion<T> q)
{
    Matrix<T, 4, 3> mat;

    float w = q.w();
    float x = q.x();
    float y = q.y();
    float z = q.z();

    mat << -x, -y, -z,
            w, -z,  y,
            z,  w, -x,
           -y,  x,  w;
    return mat;
}

#endif