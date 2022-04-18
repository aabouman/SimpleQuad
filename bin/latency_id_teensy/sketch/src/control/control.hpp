#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/latency_id_teensy/src/control/control.hpp"
#ifndef _CONTROL_HPP
#define _CONTROL_HPP

// Make sure everything is statically alloced
#define EIGEN_NO_MALLOC
#include <ArduinoEigenDense.h>
#include <Servo.h>

#define LQR_NUM_STATES 13
#define LQR_NUM_ERR_STATES 12
#define LQR_NUM_INPUTS 4

#define U_HOVER 1427.3354062457752

#define MIN_THROTTLE 1148.0
#define MAX_THROTTLE 1832.0

using namespace Eigen;

namespace Control
{
    typedef struct _MOTORS
    {
        Servo front_left_esc;
        Servo front_right_esc;
        Servo back_right_esc;
        Servo back_left_esc;
    } motors_t;

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
    const state_t<float> x_hover(-0.02, 0.17, 1.70, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    const input_t<float> u_hover(U_HOVER, U_HOVER, U_HOVER, U_HOVER);

    err_state_t<float> state_error(state_t<float> x2, state_t<float> x1);

    input_t<float> get_control(state_t<float> x);

    input_t<float> clamp_control(input_t<float> u);

    void command_motors(motors_t *motors, const input_t<float> &command);

    void kill(motors_t *motors);

    void calibrate(motors_t *motors);

    void arm(motors_t *motors);

    void initialize_motors(motors_t *motors,
                           int front_left_pin,
                           int front_right_pin,
                           int back_right_pin,
                           int back_left_pin);
}

#endif