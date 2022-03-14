#include <iostream>

#include <ArduinoEigenDense.h>

#include "../src/kalman/quad_model/quad_model.h"

using namespace Eigen;

int main(int argc, char *argv[])
{
    std::cout << ("Running dynamics and process");

    state<float> x = state<float>::Random(EKF_NUM_STATES);
    input<float> u = input<float>::Random(EKF_NUM_INPUTS);
    float dt = 0.1;

    process(x, u, dt);

    std::cout << ("Ran Dynamics");
    std::cout << "Computing Jacobian";

    test_autodiff_jacobian();

    return 0;
}