#ifndef IMU_VICON_RELAY_HPP
#define IMU_VICON_RELAY_HPP

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "pose.hpp"
#include <Wire.h>
#include <SPI.h>

#define SENSOR_ID -1
#define IMU_ADDRESS 0x28

#define RF95_FREQ 915E6
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

typedef struct tmp_IMU_VICON
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
} IMU_VICON;

#define IMU_VICON_init_zero                      \
    {                                            \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 \
    }

constexpr int POSE_MSG_SIZE = sizeof(rexlab::Pose<int16_t>);

class ImuViconRelay
{
private:
    // IMU support attributes
    sensors_event_t _imu_event;
    Adafruit_BNO055 _bno;
    // Vicon/LoRa support attributes
    uint8_t _lora_buffer[POSE_MSG_SIZE];
    rexlab::Pose<int16_t> _vicon_int16;
    rexlab::Pose<float> _vicon_float;
    // New message flags
    bool _new_vicon;

    // void onLoRaReceive(int packetSize);

public:
    ImuViconRelay(int32_t imu_sensor_id,
                  uint8_t imu_address,
                  TwoWire *imu_wire,
                  uint8_t lora_cs,
                  uint8_t lora_rst,
                  uint8_t lora_int);
    ~ImuViconRelay();

    bool hasReceived();

    void updateVicon(IMU_VICON * imu_vicon);
    void updateImu(IMU_VICON &imu_vicon);

    bool calibrateImu();
};

bool constraintCheck(IMU_VICON &imu_vicon);
void displayImuVicon(IMU_VICON &imu_vicon);

#endif