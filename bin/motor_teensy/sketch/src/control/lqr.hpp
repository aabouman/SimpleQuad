#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/src/control/lqr.hpp"
#ifndef LQR_HPP
#define LQR_HPP

#include <ArduinoEigenDense.h>
#include "quaternion_diff.h"

// Make sure everything is statically alloced
#define EIGEN_NO_MALLOC

#define LQR_NUM_STATES 13
#define LQR_NUM_ERR_STATES 12
#define LQR_NUM_INPUTS 4

#define U_HOVER 1427.3354062457752
#define MAX_THROTTLE 1148.0
#define MIN_THROTTLE 1832.0

namespace Control
{

// State Type
template <typename T>
using state_t = Matrix<T, LQR_NUM_STATES, 1>;
// State Type
template <typename T>
using err_state_t = Matrix<T, LQR_NUM_ERR_STATES, 1>;
// State Type
template <typename T>
using input_t = Matrix<T, LQR_NUM_INPUTS, 1>;

// Populate the hover state/input
const state_t<float> x_hover(0.150, .150, 0.05, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
const input_t<float> u_hover(U_HOVER, U_HOVER, U_HOVER, U_HOVER);

err_state_t<float> state_error(state_t<float> x2, state_t<float> x1)
{
    Vector3<float> pos1 = x1.segment(0, 3);
    Quaternion<float> quat1(x1(3), x1(4), x1(5), x1(6));
    Vector3<float> vel1 = x1.segment(7, 3);
    Vector3<float> omg1 = x1.segment(10, 3);

    Vector3<float> pos2 = x2.segment(0, 3);
    Quaternion<float> quat2(x2(3), x2(4), x2(5), x2(6));
    Vector3<float> vel2 = x2.segment(7, 3);
    Vector3<float> omg2 = x2.segment(10, 3);

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
    Matrix<float, LQR_NUM_ERR_STATES, LQR_NUM_INPUTS> K_gains;
    K_gains << 1.3157338657427822, 1.386613052699102, -1.3157338312380813, -1.3866130872053368,
        1.386828075025603, -1.3160464714865245, -1.3868280419892471, 1.3160464384710817,
        1.5761396117654949, 1.5761395596270469, 1.5761396118606046, 1.5761395597945393,
        37.857604372152814, -39.08089236857682, -37.857603301361536, 39.080891297871716,
        -39.04251033362917, -37.826528282438495, 39.04250946604728, 37.82652915003015,
        139374.21218251894, -139374.2124593671, 139374.21236163398, -139374.21198806437,
        139374.21218251903, -139374.2124593673, 139374.21236163398, -139374.2119880645,
        1.879403335510136, 1.7927295724179826, -1.879403287057985, -1.792729620870721,
        1.7933542281946833, -1.8800935918255477, -1.7933541701560558, 1.8800935337942901,
        10.213494130331817, 10.213493811366945, 10.213494130707419, 10.213493812268718,
        3.240037295509049, -3.2437980272563722, -3.2400372482172206, 3.243797979967574,
        -3.2383033408219806, -3.2360464984555537, 3.238303304761369, 3.236046534516526;

    input_t<float> u = u_hover + K_gains.transpose() * state_error(x, x_hover);
    return u;
}

input_t<float> clamp_control(input_t<float> u)
{
    return u.cwiseMin(MAX_THROTTLE).cwiseMax(MIN_THROTTLE);
}

}

#endif