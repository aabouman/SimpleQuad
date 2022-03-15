#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <PacketSerial.h>
#include <crc8.h>

#include "src/kalman/kalman.hpp"
#include "src/control/lqr.hpp"
#include "utils.hpp"

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define LED_PIN 13

imu_vicon_t data = {0,0,0, 1,0,0,0, 0,0,0, 0,0,0, 0};
crc8_params params = DEFAULT_CRC8_PARAMS;
PacketSerial featherPacketSerial;

// Setting up the filter
float dt = 0.1;
Filter::EKF ekf = Filter::EKF(0.1, 0.1);
Filter::state_t<float> filt_state(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
Filter::input_t<float> filt_input(0, 0, 0, 0, 0, 0);
Filter::measurement_t<float> filt_meas(0, 0, 0, 1, 0, 0, 0);

// Setting up the controller
Control::state_t<float> cont_state(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
Control::input_t<float> cont_input(0, 0, 0, 0);

// Startup
#line 29 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
void setup();
#line 53 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
void loop();
#line 79 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
void onPacketReceived(const uint8_t *buffer, size_t bytes_recv);
#line 29 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
void setup()
{
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }
    Serial3.begin(115200);
    while (!Serial3)
    {
        delay(10);
    }
    Serial.println("Started Serial3");

    // Setting the initial state of the quadrotor
    ekf.set_state(filt_state);

    // Setup the packet serial
    featherPacketSerial.setStream(&Serial3);
    featherPacketSerial.setPacketHandler(&onPacketReceived);
}

void loop()
{
    delay(10);
    featherPacketSerial.update();
    imu_vicon_to_filt(data, &filt_input, &filt_meas);

    // Convert filter state to control state
    Filter::state_t<float> tmp_state = ekf.get_state();
    filt_to_cont(tmp_state, filt_input, &cont_state);
    // Compute inputs from th econtroller state
    cont_input = Control::get_control(cont_state);
    Serial.printf("u: [%.4f,  %.4f,  %.4f,  %.4f]\n", cont_input(0), cont_input(1), cont_input(2), cont_input(3));

    // Run filter step
    ekf.prediction(filt_input, dt);
    ekf.update(filt_meas);

    if (featherPacketSerial.overflow())
    {
        digitalWrite(LED_PIN, HIGH);
    }
}

/*
 * Relay the message over from LoRa/IMU to Jetson
 */
void onPacketReceived(const uint8_t *buffer, size_t bytes_recv)
{
    uint8_t crc8_byte_recv = buffer[bytes_recv - 1];
    uint8_t crc8_byte_comp = crc8(params, (const uint8_t *)buffer, bytes_recv - 1);

    if (crc8_byte_comp == crc8_byte_recv)
    {
        memcpy(&data, &buffer, bytes_recv - 1);
    }
    else
    {
        Serial.println("Failed to parse packet with CRC8!");
    }
}

