#ifndef QUAD_TYPES_H
#define QUAD_TYPES_H

#include <ArduinoEigenDense.h>

#define EKF_NUM_STATES 16
#define EKF_NUM_ERR_STATES 15
#define EKF_NUM_INPUTS 6
#define EKF_NUM_MEASURES 7
#define EKF_NUM_ERR_MEASURES 6

using namespace Eigen;

namespace Filter
{
    // State Type
    template <typename T>
    using state_t = Matrix<T, EKF_NUM_STATES, 1>;
    // Error State Type
    template <typename T>
    using err_state_t = Matrix<T, EKF_NUM_ERR_STATES, 1>;

    // Input Type
    template <typename T>
    using input_t = Matrix<T, EKF_NUM_INPUTS, 1>;

    // Measurement Type
    template <typename T>
    using measurement_t = Matrix<T, EKF_NUM_MEASURES, 1>;
    // Error Measurement Type
    template <typename T>
    using err_measurement_t = Matrix<T, EKF_NUM_ERR_MEASURES, 1>;

    // Process Jacobian Type
    template <typename T>
    using process_jac_t = Matrix<T, EKF_NUM_STATES, EKF_NUM_STATES>;
    // Measure Jacobian Type
    template <typename T>
    using measure_jac_t = Matrix<T, EKF_NUM_MEASURES, EKF_NUM_STATES>;

    // Error State process jacobian
    template <typename T>
    using err_process_jac_t = Matrix<T, EKF_NUM_ERR_STATES, EKF_NUM_ERR_STATES>;
    // Error Measure process jacobian
    template <typename T>
    using err_measure_jac_t = Matrix<T, EKF_NUM_ERR_MEASURES, EKF_NUM_ERR_STATES>;

    // Jacobian mapping a state into an error state
    template <typename T>
    using err_state_state_jac_t = Matrix<T, EKF_NUM_ERR_STATES, EKF_NUM_STATES>;
    // Jacobian mapping a measurement into an error measurement
    template <typename T>
    using err_measure_measure_jac_t = Matrix<T, EKF_NUM_ERR_MEASURES, EKF_NUM_MEASURES>;

    template <typename T>
    using process_cov_t = Matrix<T, EKF_NUM_ERR_STATES, EKF_NUM_ERR_STATES>;
    template <typename T>
    using measure_cov_t = Matrix<T, EKF_NUM_ERR_MEASURES, EKF_NUM_ERR_MEASURES>;
}

#endif