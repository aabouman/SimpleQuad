#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/latency_id_teensy/src/kalman/util/eigen_utils.hpp"
#ifndef _EIGEN_UTILS_HPP
#define _EIGEN_UTILS_HPP

#include <ArduinoEigenDense.h>
#define EIGEN_NO_MALLOC

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char *sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif // __arm__


using namespace Eigen;

/*
 * Helper function to "pretty" print the matrix X
 */
// template <int num_rows, int num_cols>
// void print_matrix(const Matrix<float, num_rows, num_cols> &X)
// {
//     int i, j;

//     for (i = 0; i < num_rows; i++)
//     {
//         Serial.print("| ");
//         for (j = 0; j < num_cols - 1; j++)
//         {
//             Serial.printf("%+.3g, ", X(i, j));
//         }
//         Serial.printf("%+.3g |\n", X(i, j));
//     }
// }

template <int num_rows, int num_cols>
void print_matrix(const Matrix<float, num_rows, num_cols> &X)
{
    int i, j;

    for (i = 0; i < num_rows; i++)
    {
        printf("| ");
        for (j = 0; j < num_cols - 1; j++)
        {
            printf("%+1.3f, ", X(i, j));
        }
        printf("%+1.3f |\n", X(i, j));
    }
}

#endif