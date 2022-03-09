# 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino"
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

# 17 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino" 2
# 18 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino" 2
# 19 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino" 2

// #include "crc8.h"
// #include "matrix.h"
// #include "linalg.h"



using namespace Eigen;

// Vicon
// constexpr int POSE_MSG_SIZE = sizeof(rexlab::Pose<int16_t>);
// uint8_t lora_buffer[POSE_MSG_SIZE];

// Build buffers and message types
// constexpr int IMU_VICON_MSG_SIZE = sizeof(IMU_VICON) + 1;
// uint8_t imu_vicon_buffer[IMU_VICON_MSG_SIZE];

// Message type
// IMU_VICON imu_vicon = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0};
// ImuViconRelay *relay;

// Initialize packet serial ports
// void sendJetsonMessage(IMU_VICON &imu_vicon);
// CRC8_PARAMS crc8_params = DEFAULT_CRC8_PARAMS;

// DiagonalMatrix<float, EKF_NUM_ERR_STATES> Q_cov;
// DiagonalMatrix<float, EKF_NUM_ERR_MEASURES> R_cov;

// Startup
void setup()
{
    pinMode(13, (0x1));
    // Setup PacketSerial to handle communicating from Serial
    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }

    // float dt = 0.01;
    EKF ekf = EKF();

    // State curr_state(0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0);
    // Input curr_input(0,0,0,0,0,0);

    // ekf.process(curr_state, curr_input, dt);

    // Initialize IMU VICON Relay and point to it with global
    // ImuViconRelay tmp = ImuViconRelay();
    // relay = &tmp;
}

void loop()
{
    delay(100);
    // Serial.println("Test");
    // // Limit to 10 Hz
    // // delay(100);

    // If LoRa has received update vicon entry
    // bool flag = (*relay).hasReceived();
    // Serial.printf("Are we going into loop?: %d\n", flag);

    // if ((*relay).hasReceived())
    // {
    //     Serial.println("going to update Vicon");

    //     (*relay).updateVicon(imu_vicon);
    // }
    // // Update imu entry
    // (*relay).updateImu(imu_vicon);
    // displayImuVicon(imu_vicon);

    // Serial.println(POSE_MSG_SIZE);
    // // Send IMU/Vicon Message
    // // Serial.printf(" Quat: [%1.3f, %1.3f, %1.3f, %1.3f]\n", imu_vicon.quat_w, imu_vicon.quat_x, imu_vicon.quat_y, imu_vicon.quat_z);
    // sendJetsonMessage(imu_vicon);
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
