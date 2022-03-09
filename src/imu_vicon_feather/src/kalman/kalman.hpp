#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <ArduinoEigenDense.h>

#define EKF_NUM_STATES          16
#define EKF_NUM_ERR_STATES      15
#define EKF_NUM_INPUTS          6
#define EKF_NUM_MEASURES        7
#define EKF_NUM_ERR_MEASURES    6

using namespace std;
using namespace Eigen;

typedef Matrix<float, EKF_NUM_STATES, 1> State;
typedef Matrix<float, EKF_NUM_ERR_STATES, 1> ErrState;
typedef Matrix<float, EKF_NUM_INPUTS, 1> Input;
typedef Matrix<float, EKF_NUM_MEASURES, 1> Measurement;
typedef Matrix<float, EKF_NUM_ERR_MEASURES, 1> ErrMeasurement;

class EKF
{

private:
    // Initialize vectors for storing the state and error states
    State _est_state;
    ErrState _err_est_state_cov;
    // Initialize vectors for storing the inputs and error inputs
    Input _input;
    // Initialize vectors for storing the inputs and error measurements
    Measurement _measurement;
    ErrMeasurement _err_measurement;
    // Process and Measure Covariance
    DiagonalMatrix<float, EKF_NUM_ERR_STATES> _Q_cov;
    DiagonalMatrix<float, EKF_NUM_ERR_MEASURES> _R_cov;

    // Model process function (dynamics)
    // State process(State &curr_state, Input &curr_input, float dt);
    // Model measure function (sensing model)
    Measurement measure(State &curr_state);
    // Jacobian mapping state into next "error" state
    Matrix<float, EKF_NUM_ERR_STATES, EKF_NUM_STATES> error_process_jacobian(State &curr_state, Input &curr_input, float dt);
    // Jacobian mapping measurement into "error" measurement
    Matrix<float, EKF_NUM_ERR_MEASURES, EKF_NUM_MEASURES> error_measure_jacobian(State &curr_state);
    // function adding states and "error" state
    State state_composition(State &state, ErrState &err_state);
    // function taking the difference between two measurements as error measurement
    ErrMeasurement measurement_error(Measurement &measurement1, Measurement &measurement2);
    // Compute the innovation
    // innovation(Matrix<double, _num_states, 1> &state_k2_k1,
    //            Matrix<double, _num_err_states, _num_err_states> &cov_k2_k1, );

public:
    EKF();
    // EKF(const DiagonalMatrix<float, EKF_NUM_ERR_STATES, EKF_NUM_ERR_STATES> &Q,
    //     const DiagonalMatrix<float, EKF_NUM_ERR_MEASURES, EKF_NUM_ERR_MEASURES> &R);
    ~EKF();
    // Model process function (dynamics)
    State process(State &curr_state, Input &curr_input, float dt);

    void prediction(Input &input);
    void update(Measurement &measurement);
};

#endif
