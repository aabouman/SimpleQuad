#include <Arduino.h>
#include <PacketSerial.h>
#include <crc8.h>

#include "src/imu_vicon/imu_vicon.hpp"

// #define DEBUG
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define LED_PIN 13

void sendTeensyMessage(imu_vicon_t &data);

imu_vicon_t data = imu_vicon_init_zero;
crc8_params params = DEFAULT_CRC8_PARAMS;
PacketSerial teensyPacketSerial;

bool led_state = LOW;

// Startup
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, led_state);

#ifdef DEBUG
    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }
    Serial.println("Serial started");
#endif

    // Initialize IMU VICON Relay and point to it with global
    init_imuViconRelay();

    Serial1.begin(115200);
    while (!Serial1)
    {
        delay(10);
    }
    if (Serial)
    {
        Serial.println("Serial1 started");
    }

    teensyPacketSerial.setStream(&Serial1);
}

void loop()
{
    // Limit to 100 Hz
    delay(10);
    teensyPacketSerial.update();

    // Update imu entry
    updateIMU(&data);

    // If LoRa has received update vicon entry and send onto the Teensy
    if (hasLoRaReceived())
    {
        updateVicon(&data);
        // Send info to Teensy
        sendTeensyMessage(data);
    }

#ifdef DEBUG
    displayImuVicon(&data);
#endif
}

/*
 * Relay the message over from LoRa/IMU to Jetson
 */
void sendTeensyMessage(imu_vicon_t &data)
{
    // Copy IMU_VICON message into buffer of size sizeof(IMU_VICON)+1
    size_t imu_vicon_size = sizeof(imu_vicon_t);
    uint8_t buffer[imu_vicon_size + 1];

    memcpy(buffer, &data, imu_vicon_size);
    // Compute the CRC8 value of the IMU_VICON message
    uint8_t crc = crc8(params, buffer, imu_vicon_size);
    buffer[imu_vicon_size] = crc;
    // Write IMU_VICON value along with crc8 value
    teensyPacketSerial.send(buffer, imu_vicon_size + 1);

    digitalWrite(LED_PIN, led_state);
    led_state = !led_state;
}
