#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <PacketSerial.h>
#include <crc8.h>

#include "src/kalman/kalman.hpp"
#include "src/control/lqr.hpp"
#include "utils.hpp"

#define DEBUG
#define DEBUG_PRINT(str)         \
    {                            \
        if (Serial)              \
        {                        \
            Serial.println(str); \
        }                        \
    }
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define LED_PIN 13

void onPacketReceived(const uint8_t *buffer, size_t bytes_recv);

imu_vicon_t data = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
crc8_params decode_params = DEFAULT_CRC8_PARAMS;
PacketSerial featherPacketSerial;
bool new_imu_vicon = false;

// Setting up the filter
int time = micros();
int last_time = micros();
Filter::EKF ekf = Filter::EKF(0.1, 0.1);
Filter::state_t<float> filt_state(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
Filter::input_t<float> filt_input(0, 0, 0, 0, 0, 0);
Filter::measurement_t<float> filt_meas(0, 0, 0, 1, 0, 0, 0);

// Setting up the controller
Control::state_t<float> cont_state(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
Control::input_t<float> cont_input(0, 0, 0, 0);

// Startup
#line 41 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
void setup();
#line 69 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
void loop();
#line 41 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/motor_teensy.ino"
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

    Serial3.begin(115200);
    while (!Serial3)
    {
        delay(10);
    }
    DEBUG_PRINT("Started Serial3");

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



    if (new_imu_vicon)
    {
        DEBUG_PRINT("Loop");

        // Run filter step
        float dt = (time - last_time) / 1e6;
        // imu_vicon_to_filt(data, &filt_input, &filt_meas);
        // ekf.prediction(filt_input, dt);
        // ekf.update(filt_meas);

        // // Convert filter state to control state
        // Filter::state_t<float> tmp_state = ekf.get_state();
        // filt_to_cont(tmp_state, filt_input, &cont_state);

        DEBUG_PRINT(dt);

        cont_state = imu_integrator(data, dt);
        print_matrix(cont_state);

        // Compute inputs from th econtroller state
        cont_input = Control::get_control(cont_state);
        cont_input = Control::clamp_control(cont_input);
        Serial.printf("u: [%.4f,  %.4f,  %.4f,  %.4f]\n",
                      cont_input(0), cont_input(1),
                      cont_input(2), cont_input(3));
        //Command motors

        new_imu_vicon = false;
    }

    //
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
    uint8_t crc8_byte_comp = crc8(decode_params, (const uint8_t *)buffer, bytes_recv - 1);
    size_t bytes_expected = sizeof(imu_vicon_t) + 1;

    Serial.printf("bytes_recv: %d\n", bytes_recv);
    // Serial.printf("bytes expected: %d\n", sizeof(imu_vicon_t) + 1);

    if ((crc8_byte_comp == crc8_byte_recv) && (bytes_recv == bytes_expected))
    {
        memcpy(&data, buffer, sizeof(imu_vicon_t));
        new_imu_vicon = true;
        last_time = time;
        time = micros();
        // DEBUG_PRINT("Heard packet!");

        // /* Display the individual values */
        // Serial.println("\n-------------Sensor Reading-------------");
        // Serial.printf(" Acc: [%1.3f, %1.3f, %1.3f]\n", data.acc_x, data.acc_y, data.acc_z);
        // Serial.printf(" Gyr: [%1.3f, %1.3f, %1.3f]\n", data.gyr_x, data.gyr_y, data.gyr_z);
        // Serial.printf(" Pos: [%1.3f, %1.3f, %1.3f]\n", data.pos_x, data.pos_y, data.pos_z);
        // Serial.printf(" Quat: [%1.3f, %1.3f, %1.3f, %1.3f]\n", data.quat_w, data.quat_x, data.quat_y, data.quat_z);
        // Serial.println("\n----------------------------------------");

    }
    else
    {
        DEBUG_PRINT("Failed to parse packet with CRC8!");
    }
}

