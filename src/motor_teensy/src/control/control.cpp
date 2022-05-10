#include "control.hpp"
#include "quaternion_diff.hpp"

// #define EIGEN_NO_MALLOC
// #include <ArduinoEigenDense.h>

namespace Control
{
    err_state_t<float> state_error(state_t<float> x2, state_t<float> x1)
    {
        Vector3f pos1 = x1.segment(0, 3);
        Quaternionf quat1(x1(3), x1(4), x1(5), x1(6));
        Vector3f vel1 = x1.segment(7, 3);
        Vector3f omg1 = x1.segment(10, 3);

        Vector3f pos2 = x2.segment(0, 3);
        Quaternionf quat2(x2(3), x2(4), x2(5), x2(6));
        Vector3f vel2 = x2.segment(7, 3);
        Vector3f omg2 = x2.segment(10, 3);

        err_state_t<float> ret_state;
        ret_state.segment(0, 3) = pos2 - pos1;
        ret_state.segment(3, 3) = rotation_error(quat2, quat1);
        ret_state.segment(6, 3) = vel2 - vel1;
        ret_state.segment(9, 3) = omg2 - omg1;

        return ret_state;
    }

    input_t<float> get_control(state_t<float> x)
    {
        // This matrix is computed offline
        Matrix<float, LQR_NUM_INPUTS, LQR_NUM_ERR_STATES> K_gains;
        // K_gains << -1.52920, -1.52925, 1.57778, 32.16976, -32.13928, 1.57874, -2.71079, -2.71173, 7.73862, 5.30862, -5.30005, 5.64185,
        //     -1.52921, 1.52926, 1.57778, -32.17459, -32.14411, -1.57874, -2.71094, 2.71187, 7.73862, -5.30998, -5.30141, -5.64185,
        //     1.52920, 1.52925, 1.57778, -32.16976, 32.13928, 1.57874, 2.71079, 2.71173, 7.73862, -5.30862, 5.30005, 5.64185,
        //     1.52921, -1.52926, 1.57778, 32.17459, 32.14411, -1.57874, 2.71094, -2.71187, 7.73862, 5.30998, 5.30141, -5.64185;
        // K_gains << -46.57293, -46.57602, 49.41526, 140.09646, -139.94316, 4.98621, -25.98509, -25.99996, 42.54241, 11.05913, -11.04060, 10.25139,
        //     -46.57342, 46.57651, 49.41526, -140.12077, -139.96747, -4.98621, -25.98745, 26.00232, 42.54241, -11.06207, -11.04354, -10.25139,
        //     46.57293, 46.57602, 49.41526, -140.09646, 139.94316, 4.98621, 25.98509, 25.99996, 42.54241, -11.05913, 11.04060, 10.25139,
        //     46.57342, -46.57651, 49.41526, 140.12077, 139.96747, -4.98621, 25.98745, -26.00232, 42.54241, 11.06207, 11.04354, -10.25139;
        // K_gains << -46.79625, -46.79846, 49.41684, 132.40243, -132.23658, 4.98688, -25.15935, -25.17566, 42.42814, 10.36713, -10.34769, 9.75496,
        //     -46.79660, 46.79881, 49.41684, -132.42873, -132.26288, -4.98688, -25.16193, 25.17825, 42.42814, -10.37021, -10.35077, -9.75496,
        //     46.79625, 46.79846, 49.41685, -132.40243, 132.23658, 4.98688, 25.15935, 25.17566, 42.42814, -10.36713, 10.34769, 9.75496,
        //     46.79660, -46.79881, 49.41685, 132.42873, 132.26288, -4.98688, 25.16193, -25.17825, 42.42814, 10.37021, 10.35077, -9.75496;
        // K_gains << -14.93272, -14.93364, 15.71253, 86.43653, -86.35362, 4.98621, -11.94137, -11.94705, 25.61316, 8.95711, -8.94343, 10.25139,
        //     -14.93287, 14.93379, 15.71253, -86.44967, -86.36677, -4.98621, -11.94227, 11.94795, 25.61316, -8.95928, -8.94560, -10.25139,
        //     14.93272, 14.93364, 15.71253, -86.43653, 86.35362, 4.98621, 11.94137, 11.94705, 25.61316, -8.95711, 8.94343, 10.25139,
        //     14.93287, -14.93379, 15.71253, 86.44967, 86.36677, -4.98621, 11.94227, -11.94795, 25.61316, 8.95928, 8.94560, -10.25139;
        K_gains << -1.52920, -1.52925, 1.57798, 32.16976, -32.13928, 1.57874, -2.71079, -2.71173, 8.19356, 5.30862, -5.30005, 5.64185,
            -1.52921, 1.52926, 1.57798, -32.17459, -32.14411, -1.57874, -2.71094, 2.71187, 8.19356, -5.30998, -5.30141, -5.64185,
            1.52920, 1.52925, 1.57798, -32.16976, 32.13928, 1.57874, 2.71079, 2.71173, 8.19356, -5.30862, 5.30005, 5.64185,
            1.52921, -1.52926, 1.57798, 32.17459, 32.14411, -1.57874, 2.71094, -2.71187, 8.19356, 5.30998, 5.30141, -5.64185;

        input_t<float> u = u_hover - K_gains * state_error(x, x_hover);
        return u;
    }

    input_t<float> clamp_control(input_t<float> u)
    {
        return u.cwiseMin(MAX_THROTTLE).cwiseMax(MIN_THROTTLE);
    }
}

