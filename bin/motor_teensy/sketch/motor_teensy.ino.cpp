#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
#define DEBUG
#define LED_PIN 13

#define FRONT_LEFT_PIN (2)
#define FRONT_RIGHT_PIN (3)
#define BACK_RIGHT_PIN (4)
#define BACK_LEFT_PIN (5)

#include <Arduino.h>
#include "src/kalman/kalman.hpp"
#include "src/control/control.hpp"
#include "utils.hpp"

// Setting up the filter
Filter::EKF ekf = Filter::EKF(1.0, 0.1);
Filter::state_t<float> filt_state(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
Filter::input_t<float> filt_input(0, 0, 0, 0, 0, 0);
Filter::measurement_t<float> filt_meas(0, 0, 0, 1, 0, 0, 0);

// Setting up the controller
Control::state_t<float> cont_state(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
Control::input_t<float> cont_input(0, 0, 0, 0);

// Setting up the motor command structure
Control::motors_t motors;
int last_command_time = micros();

// Startup
#line 29 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
void setup();
#line 56 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
void loop();
#line 29 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
void setup()
{
    pinMode(LED_PIN, OUTPUT);

#ifdef DEBUG
    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }
    Serial.println("Started Serial");
#endif

    // Start up comunications with feather
    initFeatherPacket();

    // Setting the initial state of the quadrotor
    ekf.set_state(filt_state);

    // Setup Motor ESC
    DEBUG_PRINT("Initializing motors");
    Control::initialize_motors(&motors, FRONT_LEFT_PIN, FRONT_RIGHT_PIN,
                               BACK_RIGHT_PIN, BACK_LEFT_PIN);

    while (getFeatherPacket(&filt_input, &filt_meas) <= 0);
}

void loop()
{
    float dt = getFeatherPacket(&filt_input, &filt_meas);

    if (dt > 0.0)
    {
        // Run mEKF
        ekf.prediction(filt_input, dt);
        ekf.update(filt_meas);

        // Compute inputs from th econtroller state
        Filter::state_t<float> tmp_state = ekf.get_state();
        filt_to_cont(tmp_state, filt_input, &cont_state);
        cont_input = Control::get_control(cont_state);
        cont_input = Control::clamp_control(cont_input);

#ifdef DEBUG
        Serial.println("Filter State");
        Serial.printf("\tp: [%+1.4f,  %+1.4f, %+1.4f]\n\tq: [%+1.4f,  %+1.4f, %+1.4f, %+1.4f]\n\tv: [%+1.4f,  %+1.4f, %+1.4f]\n\ta: [%+1.4f,  %+1.4f, %+1.4f]\n\tb: [%+1.4f,  %+1.4f, %+1.4f]\n",
                      tmp_state(0), tmp_state(1), tmp_state(2),
                      tmp_state(3), tmp_state(4), tmp_state(5), tmp_state(6),
                      tmp_state(7), tmp_state(8), tmp_state(9),
                      tmp_state(10), tmp_state(11), tmp_state(12),
                      tmp_state(13), tmp_state(14), tmp_state(15));
        Serial.println("Controller State");
        Serial.printf("\tp: [%+1.4f,  %+1.4f, %+1.4f]\n\tq: [%+1.4f,  %+1.4f, %+1.4f, %+1.4f]\n\tv: [%+1.4f,  %+1.4f, %+1.4f]\n\tw: [%+1.4f,  %+1.4f, %+1.4f]\n",
                      cont_state(0), cont_state(1), cont_state(2),
                      cont_state(3), cont_state(4), cont_state(5), cont_state(6),
                      cont_state(7), cont_state(8), cont_state(9),
                      cont_state(10), cont_state(11), cont_state(12));
        Control::err_state_t<float> error = Control::state_error(cont_state, Control::x_hover);
        Serial.println("State Error From Hover");
        Serial.printf("\tdp: [%+1.4f,  %+1.4f, %+1.4f]\n\tdo: [%+1.4f,  %+1.4f, %+1.4f]\n\tdv: [%+1.4f,  %+1.4f, %+1.4f]\n\tdw: [%+1.4f,  %+1.4f, %+1.4f]\n",
                      error(0), error(1), error(2),
                      error(3), error(4), error(5),
                      error(6), error(7), error(8),
                      error(9), error(10), error(11));
        Serial.println("Controller Input");
        Serial.printf("\tu: [%+1.4f,  %+1.4f,  %+1.4f,  %+1.4f]\n",
                      cont_input(0), cont_input(1),
                      cont_input(2), cont_input(3));
#endif
        // Command motors
        Control::command_motors(&motors, cont_input);
        last_command_time = micros();
    }

    // Kill if haven't received a message in awhile
    float time_since_last_command = (micros() - last_command_time) / 1E6;
    if (time_since_last_command > 0.1)
    {
        kill(&motors);
    }
}

