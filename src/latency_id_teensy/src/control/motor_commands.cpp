#include "control.hpp"

namespace Control
{
    static bool valid_command(const input_t<float> &command)
    {
        bool valid = (MIN_THROTTLE <= command(0) && command(0) <= MAX_THROTTLE) &&
                     (MIN_THROTTLE <= command(1) && command(1) <= MAX_THROTTLE) &&
                     (MIN_THROTTLE <= command(2) && command(2) <= MAX_THROTTLE) &&
                     (MIN_THROTTLE <= command(3) && command(3) <= MAX_THROTTLE);
        return valid;
    }

    void command_motors(motors_t *motors, const input_t<float> &command)
    {
        if (valid_command(command))
        {
            motors->front_left_esc.writeMicroseconds((int) command(0));
            motors->front_right_esc.writeMicroseconds((int) command(1));
            motors->back_right_esc.writeMicroseconds((int) command(2));
            motors->back_left_esc.writeMicroseconds((int) command(3));
        }
    }

    void kill(motors_t *motors)
    {
        motors->front_left_esc.writeMicroseconds(MIN_THROTTLE);
        motors->front_right_esc.writeMicroseconds(MIN_THROTTLE);
        motors->back_right_esc.writeMicroseconds(MIN_THROTTLE);
        motors->back_left_esc.writeMicroseconds(MIN_THROTTLE);
    }

    void calibrate(motors_t *motors)
    {
        motors->front_left_esc.writeMicroseconds(MAX_THROTTLE);
        motors->front_right_esc.writeMicroseconds(MAX_THROTTLE);
        motors->back_right_esc.writeMicroseconds(MAX_THROTTLE);
        motors->back_left_esc.writeMicroseconds(MAX_THROTTLE);
        delay(7000);
        motors->front_left_esc.writeMicroseconds(MIN_THROTTLE);
        motors->front_right_esc.writeMicroseconds(MIN_THROTTLE);
        motors->back_right_esc.writeMicroseconds(MIN_THROTTLE);
        motors->back_left_esc.writeMicroseconds(MIN_THROTTLE);
        delay(8000);
    }

    void arm(motors_t *motors)
    {
        motors->front_left_esc.writeMicroseconds(MAX_THROTTLE);
        motors->front_right_esc.writeMicroseconds(MAX_THROTTLE);
        motors->back_right_esc.writeMicroseconds(MAX_THROTTLE);
        motors->back_left_esc.writeMicroseconds(MAX_THROTTLE);
        delay(2000);

        motors->front_left_esc.writeMicroseconds(MIN_THROTTLE + 200);
        motors->front_right_esc.writeMicroseconds(MIN_THROTTLE + 200);
        motors->back_right_esc.writeMicroseconds(MIN_THROTTLE + 200);
        motors->back_left_esc.writeMicroseconds(MIN_THROTTLE + 200);
        delay(2000);
    }

    void initialize_motors(motors_t *motors,
                               int front_left_pin,
                               int front_right_pin,
                               int back_right_pin,
                               int back_left_pin)
    {
        motors->front_left_esc.attach(front_left_pin, MIN_THROTTLE, MAX_THROTTLE);
        motors->front_right_esc.attach(front_right_pin, MIN_THROTTLE, MAX_THROTTLE);
        motors->back_right_esc.attach(back_right_pin, MIN_THROTTLE, MAX_THROTTLE);
        motors->back_left_esc.attach(back_left_pin, MIN_THROTTLE, MAX_THROTTLE);

        delay(1000);
        calibrate(motors);
        delay(1000);
    }

}
