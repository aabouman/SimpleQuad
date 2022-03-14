#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/src/kalman/kalman.cpp"
// #include "kalman.hpp"

// #include <ArduinoEigenDense.h>
// // #include <ArduinoEigen/Eigen/Geometry>

// #include "kalman.hpp"
// #include "util/quad_model.h"


// using namespace Eigen;

// EKF::EKF()
// {
//     float rho = 0.1;
//     _Q_cov.diagonal() << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
//     _R_cov.diagonal() << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
//     _Q_cov = _Q_cov * rho;
//     _R_cov = _R_cov * rho;
// }

// EKF::~EKF() {}

// void EKF::process_step(state_t<float> &curr_state,
//                        input_t<float> &curr_input,
//                        float dt)
// {
//     this->_est_state = process(curr_state, curr_input, dt);
// }

// err_process_jac_t<float> EKF::error_process_jacobian(state_t<float> &curr_state,
//                                                      input_t<float> &curr_input,
//                                                      float dt)
// {
//     err_state_state_jac_t<float> J1;
//     process_jac_t<float> A;
//     err_state_state_jac_t<float> J2;

//     A = process_jacobian(curr_state, curr_input, dt);

//     Quaternion<float> quat1(curr_state(3), curr_state(4), curr_state(5), curr_state(6));
//     J1 = err_state_state_jacobian(quat1);

//     state_t<float> next_state = process(curr_state, curr_input, dt);
//     Quaternion<float> quat2(next_state(3), next_state(4), next_state(5), next_state(6));
//     J2 = err_state_state_jacobian(quat2);

//     return J2 * A * J1.transpose();
// }

// err_measure_jac_t<float> EKF::error_measure_jacobian(state_t<float> &curr_state)
// {
//     err_state_state_jac_t<float> J;
//     measure_jac_t<float> A;
//     err_measure_measure_jac_t<float> G;

//     A = measure_jacobian(curr_state);

//     Quaternion<float> quat1(curr_state(3), curr_state(4), curr_state(5), curr_state(6));
//     J = err_state_state_jacobian(quat1);

//     measurement_t<float> meas = measure(curr_state);
//     Quaternion<float> quat2(meas(3), meas(4), meas(5), meas(6));
//     G = err_measure_measure_jacobian(quat2);

//     return G * A * J.transpose();
// }
