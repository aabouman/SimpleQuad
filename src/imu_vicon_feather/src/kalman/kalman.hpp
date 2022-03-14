#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <ArduinoEigenDense.h>

#include "util/quad_types.h"
#include "util/quad_model.h"

using namespace Eigen;

class EKF
{

private:
    // Initialize vectors for storing the state and error states
    state_t<float> _est_state;
    Matrix<float, EKF_NUM_ERR_STATES, EKF_NUM_ERR_STATES> _est_state_cov;
    // Initialize vectors for storing the inputs and error inputs
    input_t<float> _input;
    // Initialize vectors for storing the inputs and error measurements
    measurement_t<float> _measurement;
    err_measurement_t<float> _err_measurement;

    // Process and Measure Covariance
    DiagonalMatrix<float, EKF_NUM_ERR_STATES> _Q_cov;
    DiagonalMatrix<float, EKF_NUM_ERR_MEASURES> _R_cov;

    // Jacobian mapping state into next "error" state
    err_process_jac_t<float> error_process_jacobian(state_t<float> &curr_state, input_t<float> &curr_input, float dt)
    {
        err_state_state_jac_t<float> J1;
        process_jac_t<float> A;
        err_state_state_jac_t<float> J2;

        A = process_jacobian(curr_state, curr_input, dt);

        Quaternion<float> quat1(curr_state(3), curr_state(4), curr_state(5), curr_state(6));
        J1 = err_state_state_jacobian(quat1);

        state_t<float> next_state = process(curr_state, curr_input, dt);
        Quaternion<float> quat2(next_state(3), next_state(4), next_state(5), next_state(6));
        J2 = err_state_state_jacobian(quat2);

        return J2 * A * J1.transpose();
    }

    // Jacobian mapping measurement into "error" measurement
    err_measure_jac_t<float> error_measure_jacobian(state_t<float> &curr_state)
    {
        err_state_state_jac_t<float> J;
        measure_jac_t<float> A;
        err_measure_measure_jac_t<float> G;

        A = measure_jacobian(curr_state);

        Quaternion<float> quat1(curr_state(3), curr_state(4), curr_state(5), curr_state(6));
        J = err_state_state_jacobian(quat1);

        measurement_t<float> meas = measure(curr_state);
        Quaternion<float> quat2(meas(3), meas(4), meas(5), meas(6));
        G = err_measure_measure_jacobian(quat2);

        return G * A * J.transpose();
    }



public:
    EKF(){};
    // EKF(const DiagonalMatrix<float, EKF_NUM_ERR_STATES, EKF_NUM_ERR_STATES> &Q,
    //     const DiagonalMatrix<float, EKF_NUM_ERR_MEASURES, EKF_NUM_ERR_MEASURES> &R);
    ~EKF(){};

    void prediction(input_t<float> &input, float dt)
    {
        state_t<float> x1 = this->_est_state;
        Matrix<float, EKF_NUM_ERR_STATES, EKF_NUM_ERR_STATES> P = this->_est_state_cov;
        DiagonalMatrix<float, EKF_NUM_ERR_STATES> W = this->_Q_cov;

        state_t<float> x2 = process(curr_state, curr_input, dt);
        err_process_jac_t<float> A = error_process_jacobian(curr_state, curr_input, dt);

        P = A * P * A.transpose() + W;
        this->_est_state = x2;
        this->_est_state_cov = P;
    }

    void update(measurement_t<float> &meas)
    {
        state_t<float> x1 = this->_est_state;
        Matrix<float, EKF_NUM_ERR_STATES, EKF_NUM_ERR_STATES> P = this->_est_state_cov;
        DiagonalMatrix<float, EKF_NUM_ERR_STATES> R = this->_R_cov;

        measurement_t<float> y = measure(x1);
        err_measurement_t<float> z = measurement_error(meas, y);
        err_measure_jac_t<float> C = error_measure_jacobian(x1);

        ColPivHouseholderQR decomp_S(C * P * C.transpose() + R); // decompose C with a suiting decomposition
        Matrix<float, EKF_NUM_ERR_STATES, EKF_NUM_ERR_STATES> L = P * C.transpose() * decomp_S.inverse();

        state_t<float> x2 = state_composition(x1, L * z);

        Matrix<float, 3, 3> I;
        I.setIdentity();

        (I - L * C.transpose())

        // Eigen::MatrixXd A = decompC.transpose().solve(B.transpose()).transpose();

            P = A * P * A.transpose() + W;
        this->_est_state = x2;
        this->_est_state_cov = P;
    }
};

#endif
