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
Filter::EKF ekf = Filter::EKF(0.1, 0.1);
Filter::state_t<float> filt_state(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
Filter::input_t<float> filt_input(0, 0, 0, 0, 0, 0);
Filter::measurement_t<float> filt_meas(0, 0, 0, 1, 0, 0, 0);

// Setting up the controller
Control::state_t<float> cont_state(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
Control::input_t<float> cont_input(0, 0, 0, 0);

// Setting up the motor command structure
Control::motors_t motors;

// Startup
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
    Serial.println("Initializing LoRa Radio");
    initFeatherPacket();

    // Setting the initial state of the quadrotor
    Serial.println("Setting EKF");
    ekf.set_state(filt_state);

    // Setup Motor ESC
    Serial.println("Initializing motors");
    Control::initialize_motors(&motors, FRONT_LEFT_PIN, FRONT_RIGHT_PIN,
                               BACK_RIGHT_PIN, BACK_LEFT_PIN);
}

void loop()
{
    delay(10);
    float dt = getFeatherPacket(&filt_input, &filt_meas);
    // float dt = 0.0;

    if (dt > 0.0)
    {
        // // Run mEKF
        // ekf.prediction(filt_input, dt);
        // ekf.update(filt_meas);

        // Convert filter state to control state
        // Filter::state_t<float> tmp_state = ekf.get_state();
        // filt_to_cont(tmp_state, filt_input, &cont_state);

        // Run basic IMU integrator
        // cont_state = imu_integrator(data, dt);
        // print_matrix(cont_state);

        // Compute inputs from th econtroller state
        // cont_input = Control::get_control(cont_state);
        // cont_input = Control::clamp_control(cont_input);
        // Serial.printf("u: [%.4f,  %.4f,  %.4f,  %.4f]\n",
        //               cont_input(0), cont_input(1),
        //               cont_input(2), cont_input(3));

        // Command motors
        float mid_throttle = (MAX_THROTTLE + MIN_THROTTLE) / 2.0;
        Control::input_t<float> command(mid_throttle, mid_throttle, mid_throttle, mid_throttle);
        Control::command_motors(&motors, command);

        Serial.printf("u = [%f, %f, %f, %f]\n", command(0), command(1), command(2), command(3));
    }
}
