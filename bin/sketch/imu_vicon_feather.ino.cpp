#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino"
/**
 * @file IMU.ino
 * @author Alexander Bouman (alex.bouman@gmail.com)
 * @brief This script takes input from onboard LoRA and IMU over SPI and
 *        publishes the message over serial to a connected computer
 *
 * Board: LoRa Feather M0 by Adafruit
 *
 * @version 0.1
 * @date 2021-11-22
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>

#include "imu_vicon_relay.hpp"
#include "crc8.h"
#include "matrix.h"
#include "linalg.h"

#define LED_PIN     13

// IMU
// Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

// Vicon
// constexpr int POSE_MSG_SIZE = sizeof(rexlab::Pose<int16_t>);
// uint8_t lora_buffer[POSE_MSG_SIZE];

// Build buffers and message types
// constexpr int IMU_VICON_MSG_SIZE = sizeof(IMU_VICON) + 1;
// uint8_t imu_vicon_buffer[IMU_VICON_MSG_SIZE];

// Message type
IMU_VICON imu_vicon = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0};

// Initialize packet serial ports
void sendJetsonMessage(IMU_VICON &imu_vicon);
CRC8_PARAMS crc8_params = DEFAULT_CRC8_PARAMS;

// Startup
#line 47 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino"
void setup();
#line 78 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino"
void loop();
#line 47 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino"
void setup()
{
    initializeImuVicon();

    // pinMode(LED_PIN, OUTPUT);
    // // Setup PacketSerial to handle communicating from Serial
    // Serial.begin(9600);
    // while (!Serial)
    // {
    //     delay(10);
    // }

    // // Start up the lora radio
    // if (!initialize_LoRaViconReceiver(lora_buffer, POSE_MSG_SIZE))
    // {
    //     Serial.println("Failed to properly setup LoRaViconReceiver!!");
    //     while(true)
    //     {
    //     }
    // }

    // // Start up SPI IMU and initialize
    // if (!bno.begin())
    // {
    //     Serial.print("\nOoops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
    //     while (true)
    //     {
    //     }
    // }
}

void loop()
{
    // // Limit to 10 Hz
    // // delay(100);

    // // Read in VICON measurement
    // if (hasLoRaReceived())
    // {
    //     updateVicon(imu_vicon);
    // }

    // // Read in IMU measurement
    // updateIMU(bno, imu_vicon);

    // Serial.println(POSE_MSG_SIZE);
    // // Send IMU/Vicon Message
    // // Serial.printf(" Quat: [%1.3f, %1.3f, %1.3f, %1.3f]\n", imu_vicon.quat_w, imu_vicon.quat_x, imu_vicon.quat_y, imu_vicon.quat_z);
    // sendJetsonMessage(imu_vicon);
    // // displayImuVicon(imu_vicon);
    // // constraintCheck(imu_vicon);
}

// /*
//  * Relay the message over from LoRa/IMU to Jetson
//  */
// void sendJetsonMessage(IMU_VICON &imu_vicon)
// {
//     // Copy IMU_VICON message into buffer of size sizeof(IMU_VICON)+1
//     size_t imu_vicon_size = sizeof(IMU_VICON);
//     memcpy(imu_vicon_buffer, &imu_vicon, imu_vicon_size);
//     // Compute the CRC8 value of the IMU_VICON message
//     uint8_t crc = crc8(crc8_params, imu_vicon_buffer, imu_vicon_size);
//     imu_vicon_buffer[imu_vicon_size] = crc;
//     // Write IMU_VICON value along with crc8 value
//     Serial.write(imu_vicon_buffer, IMU_VICON_MSG_SIZE);
// }

