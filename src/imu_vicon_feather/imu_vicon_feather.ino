// #include <CRC8.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigen/unsupported/Eigen/AutoDiff>
// #include <autodiff/autodiff/forward/dual.hpp>
// #include <autodiff.h>

// #include "src/imu_vicon/imu_vicon.hpp"
#include "src/kalman/kalman.hpp"
#include "src/kalman/forward_diff.hpp"
// #include "src/kalman/kalman.hpp"
// #include "src/kalman/quat_math.h"

#define LED_PIN     13

using namespace Eigen;

// Matrix<float, 3, 1> in;
// Matrix<float, 2, 1> out;
// AutoDiffJacobian<adFunctor<float>> adjac;

// Message type
// imu_vicon data = imu_vicon_init_zero;

// Initialize packet serial ports
// void sendJetsonMessage(IMU_VICON &imu_vicon);
// CRC8_PARAMS crc8_params = DEFAULT_CRC8_PARAMS;

// DiagonalMatrix<float, EKF_NUM_ERR_STATES> Q_cov;
// DiagonalMatrix<float, EKF_NUM_ERR_MEASURES> R_cov;

// Startup
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    // Setup PacketSerial to handle communicating from Serial
    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }
    // Initialize IMU VICON Relay and point to it with global
    // init_imuViconRelay();

    // float dt = 0.01;
    // EKF ekf = EKF();
    // State curr_state(0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0);
    // Input curr_input(0,0,0,0,0,0);
    // ekf.process(curr_state, curr_input, dt);

    // in << 1, 2, 3;

    test_autodiff_jacobian();
}

void loop()
{
    // Limit to 10 Hz
    delay(100);

    // Matrix<float, 2, 3> jac;
    // adjac<float, float>(in, &out);
    // adjac(in, &out);

    // Serial.println(jac);

    // If LoRa has received update vicon entry
    // if (hasLoRaReceived())
    // {
    //     updateVicon(&data);
    // }
    // // Update imu entry
    // updateIMU(&data);
    // displayImuVicon(&data);

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
