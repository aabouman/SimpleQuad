#include <Arduino.h>
#include <PacketSerial.h>
#include <crc8.h>

#include "src/imu_vicon/imu_vicon.hpp"

// #define DEBUG
#define LED_PIN 13
#define EIGEN_NO_MALLOC

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

    calibrateIMU();

    Serial1.begin(115200);
    while (!Serial1)
    {
        delay(10);
    }
#ifdef DEBUG
    Serial.println("Serial1 started");
#endif

    teensyPacketSerial.setStream(&Serial1);
}

void loop()
{
    teensyPacketSerial.update();

    // Update imu entry
    updateIMU(&data);

    digitalWrite(LED_PIN, LOW);
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

    digitalWrite(LED_PIN, HIGH);
}
