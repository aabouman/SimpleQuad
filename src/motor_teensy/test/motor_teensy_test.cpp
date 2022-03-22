#include <iostream>

#include <ArduinoEigenDense.h>

#include "../src/kalman/kalman.hpp"

using namespace Eigen;

// template <int num_rows, int num_cols>
// void print_matrix(const Matrix<float, num_rows, num_cols> &X)
// {
//     int i, j;

//     for (i = 0; i < num_rows; i++)
//     {
//         printf("| ");
//         for (j = 0; j < num_cols - 1; j++)
//         {
//             printf("%+1.4f, ", X(i, j));
//         }
//         printf("%+1.4f |\n", X(i, j));
//     }
// }

int main(int argc, char *argv[])
{
    float rho_Q = 1.0;
    float rho_R = 0.1;
    Filter::EKF ekf = Filter::EKF(rho_Q, rho_R);

    Filter::state_t<float> x(1.0, 2.0, 3.0,
                             0.9915, 0.0698, 0.0748, 0.0798,
                             0.0795, -0.0522, -0.9441,
                             0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0);
    Filter::input_t<float> u(0.1, 0.2, 0.3, 1.4, 1.5, 1.6);
   Filter::measurement_t<float> y(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

    // Setting the initial state of the quadrotor
    ekf.set_state(x);
    float dt = 0.1;

    Filter::state_t<float> tmp_state;

    std::cout << ("Running Prediction Step\n");
    ekf.prediction(u, dt);
    print_matrix(ekf.get_state());
    print_matrix(ekf.get_cov());
    std::cout << ("Running Update Step\n");
    ekf.update(y);
    print_matrix(ekf.get_state());
    print_matrix(ekf.get_cov());

    // Quaternionf quat(1, 0, 0, 0);
    // Quaternionf omegaQuat(0.0, 1.4, 1.5, 1.6);
    // Quaternionf kin = quat * omegaQuat;
    // Vector4f kin_vec(kin.w(), kin.x(), kin.y(), kin.z());
    // kin_vec *= 1 / 2.0;

    // std::cout << ("Quaternion Kinematics\n");
    // print_matrix(kin_vec);

    return 0;
}