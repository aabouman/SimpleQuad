#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/src/kalman/eigen_utils.h"
#ifndef EIGEN_UTILS_H
#define EIGEN_UTILS_H

#include <ArduinoEigenDense.h>

using namespace Eigen;

template <int num_rows, int num_cols>
void print_mtxf(const Matrix<float, num_rows, num_cols> &X)
{
    int i, j;

    for (i = 0; i < num_rows; i++)
    {
        Serial.print("| ");
        for (j = 0; j < num_cols - 1; j++)
        {
            Serial.printf("%6f, ", X(i, j));
        }
        Serial.printf("%6f |\n", X(i, j));
    }
}

#endif