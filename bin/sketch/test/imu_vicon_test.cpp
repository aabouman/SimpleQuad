#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test/imu_vicon_test.cpp"
#include <iostream>

#include <ArduinoEigenDense.h>

#include "../src/kalman/kalman.hpp"

using namespace Eigen;

template <int num_rows, int num_cols>
void print_matrix(const Matrix<float, num_rows, num_cols> &X)
{
    int i, j;

    for (i = 0; i < num_rows; i++)
    {
        printf("| ");
        for (j = 0; j < num_cols - 1; j++)
        {
            printf("%+1.4f, ", X(i, j));
        }
        printf("%+1.4f |\n", X(i, j));
    }
}


int main(int argc, char *argv[])
{
    float rho_Q = 0.1;
    float rho_R = 0.1;
    EKF ekf = EKF(rho_Q, rho_R);

    std::cout << ("Running dynamics and process\n");

    state_t<float> x = state_t<float>::Zero(NUM_STATES);
    x(seqN(3, 4)).normalize();
    input_t<float> u = input_t<float>::Zero(NUM_INPUTS);
    measurement_t<float> y = measurement_t<float>::Zero(NUM_MEASURES);

    // Setting the initial state of the quadrotor
    ekf.set_state(x);
    float dt = 0.1;

    std::cout << ("Testing Err Process Jacobian");
    EKF::error_process_jacobian(x, u, dt);
    std::cout << ("Completed Error_Process_Jacobian\n");

    std::cout << ("Running Prediction Step");
    ekf.prediction(u, dt);
    std::cout << ("Ran Prediction Step\n");

    print_matrix(ekf.get_cov());

    return 0;
}