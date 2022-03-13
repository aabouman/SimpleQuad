#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/src/kalman/kalman.cpp"
#include "kalman.hpp"

// #include <ArduinoEigen/Eigen/Core>
#include <ArduinoEigen/Eigen/Geometry>
// #include <ArduinoEigen/Eigen/MatrixBase>

using namespace std;
using namespace Eigen;

typedef Vector4f quat_vec;
typedef Vector3f err_quat_vec;

EKF::EKF()
{
    float rho = 0.1;
    _Q_cov.diagonal() << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    _R_cov.diagonal() << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    _Q_cov = _Q_cov * rho;
    _R_cov = _R_cov * rho;
}

EKF::~EKF() {}

State dynamics(Ref<State> curr_state, Ref<Input> curr_input)
{
    // Extracting components of the state
    // Vector3f pos = curr_state({0, 1, 2});
    Quaternionf quat(curr_state({3, 4, 5, 6}));
    Vector3f vel = curr_state({7, 8, 9});
    Vector3f alpha = curr_state({10, 11, 12});
    Vector3f beta = curr_state({13, 14, 15});

    // Extracting components of the input
    Vector3f vel_dot = curr_input({1, 2, 3});
    Vector3f omega = curr_input({4, 5, 6});

    // Derivative of position
    State ret;
    ret({0, 1, 2}) = quat * vel;

    // Derivative of quaternion
    Quaternionf omegaQuat(0.0, omega(0) - beta(0), omega(1) - beta(1), omega(2) - beta(2));
    Quaternionf kin = quat * omegaQuat;
    ret(3) = 1 / 2 * kin.w();
    ret(4) = 1 / 2 * kin.x();
    ret(5) = 1 / 2 * kin.y();
    ret(6) = 1 / 2 * kin.z();
    ret.segment(3, 6) << 0, 0, 0, 0;

    // Derivative of
    Vector3f gravity(0, 0, 9.81);
    ret({7, 8, 9}, all) = vel_dot - alpha - (quat.inverse() * gravity);
    ret({10,11,12,13,14,15}).setZero();

    return ret;
}

State EKF::process(Ref<State> curr_state, Ref<Input> curr_input, float dt)
{
    // RK4 Integrator
    State tmp;
    State k1 = dynamics(curr_state, curr_input);
    tmp = curr_state + 0.5 * dt * k1;
    State k2 = dynamics(tmp, curr_input);
    tmp = curr_state + 0.5 * dt * k2;
    State k3 = dynamics(tmp, curr_input);
    tmp = curr_state + dt * k3;
    State k4 = dynamics(tmp, curr_input);
    // Set the state to the RK4 integrator output
    State next_state = curr_state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
    // Normalize Quaternion
    next_state({3, 4, 5, 6}).normalize();

    return next_state;
}

Measurement EKF::measure(Ref<State> curr_state)
{
    Measurement measurement1(curr_state.segment(0, 6));
    return measurement1;
}


// static err_quat_vec rotation_error(Ref<State> state1, Ref<State> state2)
// {

// }


// void EKF::error_process_jacobian(Ref<State> curr_state,
//                                  Ref<Input> curr_input,
//                                  float dt)
// {
// }

// void EKF::error_measure_jacobian(const Matrix<float, state_dim, 1> &curr_state)
// {

// }

// void EKF::state_composition(const Matrix<float, state_dim, 1> &state,
//                             const Matrix<float, err_state_dim, 1> &err_state)
// {

// }

// void EKF::measurement_error(const Matrix<float, _measurement, 1> &state,
//                             const Matrix<float, _measurement, 1> &state)
// {

// }

// void EKF::innovation(Matrix<double, state_dim, 1> &state_k2_k1,
//                      Matrix<double, err_state_dim, err_state_dim> &cov_k2_k1)
// {

// }