#ifndef QUATMATH_H
#define QUATMATH_H

// The Arduino max and min macros must be undef'ed to get Eigen to compile
#undef max
#undef min
#include <ArduinoEigenDense.h>
#include <ArduinoEigen/Eigen/Geometry>

using namespace Eigen;

// Quaternion utility functions

// Rotate a vector from body frame to inertial frame (assuming q is B->N)
void qrotBtoN(Ref<Vector3f> rout, const Ref<const Vector4f> qin, const Ref<const Vector3f> xin)
{
    float qs = qin(0);
    Vector3f qv = qin.tail(3);

    Vector3f tmp = xin + 2 * qv.cross(qv.cross(xin) + qs * xin);

    rout(0) = tmp(0);
    rout(1) = tmp(1);
    rout(2) = tmp(2);
}

// Rotate a vector from inertial frame to body frame (assuming q is B->N)
void qrotNtoB(Ref<Vector3f> rout, const Ref<const Vector4f> qin, const Ref<const Vector3f> xin)
{
    float qs = qin(0);
    Vector3f qv = -qin.tail(3);

    Vector3f tmp = xin + 2 * qv.cross(qv.cross(xin) + qs * xin);

    rout(0) = tmp(0);
    rout(1) = tmp(1);
    rout(2) = tmp(2);
}

// Calculate the time derivative of a quaternion given q and omega
void qdot(Ref<Vector4f> qdot, const Ref<const Vector4f> q, const Ref<const Vector3f> w)
{
    float qs = q(0);
    Vector3f qv = q.tail(3);

    qdot(0) = -0.5 * qv.dot(w);
    qdot.tail(3) = 0.5 * (qv.cross(w) + qs * w);
}

// Calcualate the vector part of the quaternion that rotates from q0->q
// This is used as the feedback term (like q-q0) in control laws
void qdif(Ref<Vector3f> d, const Ref<const Vector4f> q, const Ref<const Vector4f> q0)
{
    float qs0 = q0(0);
    Vector3f qv0 = q0.tail(3);
    float qs = q(0);
    Vector3f qv = q.tail(3);

    d = qs0 * qv - qs * qv0 - qv0.cross(qv);
}

// Calcualate the vector part of the quaternion that rotates from q0->q
// This is used as the feedback term (like q-q0) in control laws
void quaternion_error(Ref<Vector3f> quat_err, const Ref<const Vector4f> q1, const Ref<const Vector4f> q2)
{
    Quaternionf quat1(q1);
    Quaternionf quat2(q2);
    Quaternionf tmp = quat1.inverse() * quat2;
    quat_err = (1.0 / tmp.w()) * tmp.vec();
}

void quaternion_differential(Ref<Matrix<float, 4, 3>> quat_map, const Ref<const Vector4f> q)
{
    float w = q(0);
    float x = q(1);
    float y = q(2);
    float z = q(3);

    quat_map << -x, -y, -z,
                 w, -z,  y,
                 z,  w, -x,
                -y,  x,  w;
}

#endif