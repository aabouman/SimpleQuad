#ifndef imu_vicon_relay_hpp
#define imu_vicon_relay_hpp

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "pose.hpp"
#include <Wire.h>
#include <SPI.h>

#define SENSOR_ID   -1
#define IMU_ADDRESS 0x28

#define RF95_FREQ   915.0
#define RFM95_CS    8
#define RFM95_RST   4
#define RFM95_INT   3

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

class ImuViconRelay {
    private:
        // IMU support attributes
        sensors_event_t _imu_event;
        Adafruit_BNO055 _bno;
        // Vicon/LoRa support attributes
        uint8_t _lora_buffer[POSE_MSG_SIZE];
        rexlab::Pose<int16_t> _vicon_int16;
        rexlab::Pose<float> _vicon_float;
        // New message flags
        bool _new_imu;
        bool _new_vicon;

        bool onLoRaReceive(int packetSize);

    public:
        ImuViconRelay()
        ImuViconRelay(int32_t imu_sensor_id = SENSOR_ID,
                      uint8_t imu_address = IMU_ADDRESS,
                      TwoWire imu_wire = &Wire,
                      double lora_freq = RF95_FREQ,
                      uint8_t lora_cs = RFM95_CS,
                      uint8_t lora_rst = RFM95_RST,
                      uint8_t lora_int = RFM95_INT, );
        ~ImuViconRelay();

        bool hasLoRaReceived();
        bool hasImuReceived();

        void updateVicon(IMU_VICON &imu_vicon);
        void updateImu(IMU_VICON &imu_vicon);

        bool calibrateIMU();
        void displayImuVicon(IMU_VICON &imu_vicon);
};

bool constraintCheck(IMU_VICON &imu_vicon);

#endif