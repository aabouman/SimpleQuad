#ifndef EIGEN_UTILS_H
#define EIGEN_UTILS_H

#include <ArduinoEigenDense.h>

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char *sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif // __arm__

int freeMemory()
{
    char top;
#ifdef __arm__
    return &top - reinterpret_cast<char *>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
    return &top - __brkval;
#else  // __arm__
    return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif // __arm__
}

using namespace Eigen;

/*
 * Helper function to "pretty" print the matrix X
 */
template <int num_rows, int num_cols>
void print_matrix(const Matrix<float, num_rows, num_cols> &X)
{
    int i, j;

    for (i = 0; i < num_rows; i++)
    {
        Serial.print("| ");
        for (j = 0; j < num_cols - 1; j++)
        {
            Serial.printf("%+.3g, ", X(i, j));
        }
        Serial.printf("%+.3g |\n", X(i, j));
    }
}

#endif