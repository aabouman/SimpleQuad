#ifndef imu_vicon_relay_hpp
#define imu_vicon_relay_hpp

#include <cstdint>
#include "pose.hpp"

#define SENSOR_ID -1
#define IMU_ADDRESS 0x28

#define RF95_FREQ 915.0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

constexpr int POSE_MSG_SIZE = sizeof(rexlab::Pose<int16_t>);

typedef struct _IMU_VICON
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;

    float pos_x;
    float pos_y;
    float pos_z;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;

    uint32_t time;
} imu_vicon;

#define imu_vicon_init_zero                      \
    {                                            \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 \
    }

void init_imuViconRelay();

bool hasLoRaReceived();

void updateVicon(imu_vicon *data);

void updateIMU(imu_vicon *data);

void displayCalStatus();

void displaySensorReading();

bool calibrateIMU();

void displayImuVicon(imu_vicon *data);

void constraintCheck(imu_vicon *data);

#endif
