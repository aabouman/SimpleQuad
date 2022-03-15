#include <Arduino.h>
#include <PacketSerial.h>
#include <crc8.h>

#include "src/imu_vicon/imu_vicon.hpp"

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define LED_PIN 13

void sendTeensyMessage(imu_vicon_t &data);

imu_vicon_t data = imu_vicon_init_zero;
crc8_params params = DEFAULT_CRC8_PARAMS;
PacketSerial teensyPacketSerial;

bool flop = false;

// Startup
void setup()
{
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }
    Serial.println("Serial started");

    // Initialize IMU VICON Relay and point to it with global
    init_imuViconRelay();

    Serial1.begin(115200);
    while (!Serial1)
    {
        delay(10);
    }
    Serial.println("Serial1 started");

    teensyPacketSerial.setStream(&Serial1);
}

void loop()
{
    teensyPacketSerial.update();

    // Limit to 100 Hz
    delay(10);

    // If LoRa has received update vicon entry
    if (hasLoRaReceived())
    {
        updateVicon(&data);
    }
    // Update imu entry
    updateIMU(&data);

    // Send info to Teensy
    sendTeensyMessage(data);

    // Display at 1/2 the loop rate
    if (flop)
    {
        displayImuVicon(&data);
    }
    flop = !flop;
}

/*
 * Relay the message over from LoRa/IMU to Jetson
 */
void sendTeensyMessage(imu_vicon_t &data)
{
    // Copy IMU_VICON message into buffer of size sizeof(IMU_VICON)+1
    size_t imu_vicon_size = sizeof(imu_vicon_t);
    uint8_t buffer[imu_vicon_size + 1];

    memcpy(buffer, (const void *)&data, imu_vicon_size);
    // Compute the CRC8 value of the IMU_VICON message
    uint8_t crc = crc8(params, buffer, imu_vicon_size);
    buffer[imu_vicon_size] = crc;
    // Write IMU_VICON value along with crc8 value
    teensyPacketSerial.send(buffer, imu_vicon_size + 1);
}
