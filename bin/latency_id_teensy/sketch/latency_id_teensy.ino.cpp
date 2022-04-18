#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/latency_id_teensy/latency_id_teensy.ino"
#define LED_PIN 13

#define FRONT_LEFT_PIN (2)
#define FRONT_RIGHT_PIN (3)
#define BACK_RIGHT_PIN (4)
#define BACK_LEFT_PIN (5)

#include <Arduino.h>
#include <cmath>
#include "src/kalman/kalman.hpp"
#include "src/control/control.hpp"
#include "utils.hpp"

bool is_zero(Filter::measurement_t<float> &filt_meas);

// Filter types for getting feather data packet
Filter::input_t<float> filt_input(0, 0, 0, 0, 0, 0);
Filter::measurement_t<float> filt_meas(0, 0, 0, 1, 0, 0, 0);

// Setting up the motor command structure
Control::motors_t motors;
int last_command_time = micros();
int last_receive_time = micros();

// Startup
#line 26 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/latency_id_teensy/latency_id_teensy.ino"
void setup();
#line 52 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/latency_id_teensy/latency_id_teensy.ino"
void loop();
#line 26 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/latency_id_teensy/latency_id_teensy.ino"
void setup()
{
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }
    Serial.println("Started Serial");

    // Setup Motor ESC
    Serial.println("Initializing motors");
    Control::initialize_motors(&motors, FRONT_LEFT_PIN, FRONT_RIGHT_PIN,
                               BACK_RIGHT_PIN, BACK_LEFT_PIN);
    Serial.println("Motors Initialized");

    // Start up comunications with feather
    initFeatherPacket();
    while (getFeatherPacket(&filt_input, &filt_meas) <= 0);
    Serial.printf("Heard First Packet at time: %f\n", micros() / (1E6));

    // Command motor
    last_command_time = micros();
}

void loop()
{
    Control::input_t<float> cont_input(MIN_THROTTLE + 300, MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE);
    Control::command_motors(&motors, cont_input);

    float dt = getFeatherPacket(&filt_input, &filt_meas);
    if (dt > 0.0)
    {
        if (is_zero(filt_meas))
        {
            last_receive_time = micros();
            Serial.printf("Heard Zero Pos/Quat Packet at time: %f\n", last_receive_time / 1E6);
            float latency = (last_receive_time - last_command_time) / (1E6);
            Serial.printf("Latency: %f\n", latency);
            // Just kill motors forever
            while(true){
                kill(&motors);
            }
        }
    }

    // Kill if haven't received a message in awhile
    float time_since_last_receive = (micros() - last_receive_time) / 1E6;
    if (time_since_last_receive > 0.1)
    {
        kill(&motors);
    }
}

bool is_zero(Filter::measurement_t<float> &filt_meas)
{
    for (int i = 0; i < filt_meas.rows(); i++)
    {
        if (filt_meas(i) != 0.0)
        {
            return false;
        }
    }
    return true;
}
