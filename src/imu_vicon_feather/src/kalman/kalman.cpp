#include "kalman.hpp"

#include <ArduinoEigenDense.h>
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

State dynamics(const Ref<State> curr_state, const Ref<Input> curr_input)
{
    // Extracting components of the state
    Vector3f pos = curr_state.segment(0, 3);
    Quaternionf quat(curr_state.segment(3, 4));
    Vector3f vel = curr_state.segment(7, 3);
    Vector3f alpha = curr_state.segment(10, 3);
    Vector3f beta = curr_state.segment(13, 3);

    // Extracting components of the input
    Vector3f vel_dot = curr_input.segment(0, 3);
    Vector3f omega = curr_input.segment(3, 3);

    // Derivative of position
    State ret;
    ret.segment(0, 3) = quat * vel;

    // Derivative of quaternion
    Quaternionf omegaQuat(0.0, omega(0) - beta(0), omega(1) - beta(1), omega(2) - beta(2));
    Quaternionf kin = quat * omegaQuat;
    ret..segment(3, 4) = 1 / 2 * kin;

    // Derivative of linear velocity
    Vector3f gravity(0, 0, 9.81);
    ret.segment(7, 3) = vel_dot - alpha - (quat.inverse() * gravity);
    // Derivative of imu biases
    ret.segment(10, 6).setZero();

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
    next_state.segment(3, 4).normalize();
    return next_state;
}

Measurement EKF::measure(Ref<State> curr_state)
{
    Measurement measurement1(curr_state.segment(0, 6));
    return measurement1;
}

void EKF::error_process_jacobian(Matrix<double, EKF_NUM_ERR_STATES, EKF_NUM_STATES> jac,
                                 const Ref<State> curr_state,
                                 const Ref<Input> curr_input,
                                 float dt)
{


}

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