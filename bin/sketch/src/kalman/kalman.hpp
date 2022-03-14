#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/src/kalman/kalman.hpp"
#ifndef KALMAN_HPP
#define KALMAN_HPP

// Make sure everything is statically alloced
#define EIGEN_RUNTIME_NO_MALLOC

#include <ArduinoEigenDense.h>

#include "util/quad_types.h"
#include "util/quad_model.h"

using namespace Eigen;

class EKF
{

private:
    // Initialize vectors for storing the state and error states
    state_t<float> _est_state;
    process_cov_t<float> _est_state_cov;
    // Initialize vectors for storing the inputs and error measurements
    measurement_t<float> _measurement;
    err_measurement_t<float> _err_measurement;
    // Process and Measure Covariance
    process_cov_t<float> _Q_cov;
    measure_cov_t<float> _R_cov;

    // Jacobian mapping state into next "error" state
    err_process_jac_t<float> error_process_jacobian(const state_t<float> &curr_state, const input_t<float> &curr_input, float dt)
    {
        err_state_state_jac_t<float> J1;
        process_jac_t<float> A;
        err_state_state_jac_t<float> J2;

        Serial.printf("Got to line %d\n", __LINE__);

        A = process_jacobian(curr_state, curr_input, dt);

        Serial.printf("Got to line %d\n", __LINE__);

        Quaternion<float> quat1(curr_state(3), curr_state(4), curr_state(5), curr_state(6));
        J1 = err_state_state_jacobian(quat1);

        Serial.printf("Got to line %d\n", __LINE__);

        state_t<float> next_state = process(curr_state, curr_input, dt);
        Quaternion<float> quat2(next_state(3), next_state(4), next_state(5), next_state(6));
        J2 = err_state_state_jacobian(quat2);

        Serial.printf("Got to line %d\n", __LINE__);

        return J2 * A * J1.transpose();
    }

    // Jacobian mapping measurement into "error" measurement
    err_measure_jac_t<float> error_measure_jacobian(const state_t<float> &curr_state)
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
    EKF(float rho_Q, float rho_R)
    {
        _Q_cov.setIdentity();
        _R_cov.setIdentity();

        _Q_cov = _Q_cov * rho_Q;
        _R_cov = _R_cov * rho_R;
    };

    ~EKF(){};

    void set_state(const state_t<float> &curr_state)
    {
        this->_est_state = curr_state;
    }


    void prediction(const input_t<float> &input, float dt)
    {

        Serial.printf("Got to line %d\n", __LINE__);

        state_t<float> x1 = this->_est_state;
        process_cov_t<float> P = this->_est_state_cov;
        process_cov_t<float> W = this->_Q_cov;

        Serial.printf("Got to line %d\n", __LINE__);

        state_t<float> x2 = process(x1, input, dt);

        Serial.printf("Got to line %d\n", __LINE__);

        err_process_jac_t<float> A = error_process_jacobian(x1, input, dt);

        Serial.printf("Got to line %d\n", __LINE__);

        P = A * P * A.transpose() + W;
        this->_est_state = x2;
        this->_est_state_cov = P;

        Serial.printf("Got to line %d\n", __LINE__);
    }

    void update(const measurement_t<float> &meas)
    {
        state_t<float> x1 = this->_est_state;
        process_cov_t<float> P = this->_est_state_cov;
        measure_cov_t<float> R = this->_R_cov;

        measurement_t<float> y = measure(x1);
        err_measurement_t<float> z = measurement_error(meas, y);
        err_measure_jac_t<float> C = error_measure_jacobian(x1);

        ColPivHouseholderQR<measure_cov_t<float>> decomp_S(C * P * C.transpose() + R); // decompose C with a suiting decomposition
        Matrix<float, NUM_ERR_STATES, NUM_ERR_MEASURES> L = P * C.transpose() * decomp_S.inverse();

        state_t<float> x2 = state_composition(x1, L * z);

        // Joseph form covariance update
        process_cov_t<float> I = process_cov_t<float>::Identity();
        // I.setIdentity();
        process_cov_t<float> A = I - L * C;
        P = A * P * A.transpose() + L * R * L.transpose();

        // Update attributes
        this->_est_state = x2;
        this->_est_state_cov = P;
    }
};

#endif
