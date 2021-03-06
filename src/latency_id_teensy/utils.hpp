#ifndef _UTILS_HPP
#define _UTILS_HPP

#define DEBUG_PRINT(str)         \
    {                            \
        if (Serial)              \
        {                        \
            Serial.println(str); \
        }                        \
    }
#define EIGEN_NO_MALLOC

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigen/Eigen/Geometry>
#include <PacketSerial.h>
#include <crc8.h>
#include <Servo.h>

#include "src/kalman/kalman.hpp"
#include "src/control/control.hpp"

using namespace Eigen;

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
} imu_vicon_t;

imu_vicon_t data = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Vector3f last_pos = Vector3f::Zero();
crc8_params decode_params = DEFAULT_CRC8_PARAMS;
PacketSerial featherPacketSerial;
bool new_imu_vicon = false;
int time = micros();
int last_time = micros();

void filt_to_cont(Filter::state_t<float> &filt_state,
                  Filter::input_t<float> &filt_input,
                  Control::state_t<float> *cont_state)
{
    Control::state_t<float> &v = *cont_state;

    // Extract components from state
    Vector3f pos = filt_state(seqN(0, 3));
    Vector4f quat = filt_state(seqN(3, 4));
    Vector3f vel = filt_state(seqN(7, 3));
    Vector3f beta = filt_state(seqN(13, 3));
    // Extract components from imu/vicon measurement
    Vector3f omega = filt_input(seqN(3, 3));

    v(seqN(0, 3)) = pos;
    v(seqN(3, 4)) = quat;
    v(seqN(7, 3)) = vel;
    v(seqN(10, 3)) = omega + beta;
}

void imu_vicon_to_filt(imu_vicon_t &data,
                       Filter::input_t<float> *filt_input,
                       Filter::measurement_t<float> *filt_meas)
{
    *filt_input = Matrix<float, 6, 1>(data.acc_x, data.acc_y, data.acc_z,
                                      data.gyr_x, data.gyr_y, data.gyr_z);
    *filt_meas = Matrix<float, 7, 1>(data.pos_x, data.pos_y, data.pos_z,
                                     data.quat_w, data.quat_x, data.quat_y, data.quat_z);
}

void displayImuVicon(imu_vicon_t *data)
{
    if (Serial)
    {
        /* Display the individual values */
        Serial.println("\n-------------Sensor Reading-------------");
        Serial.printf(" Acc: [%1.3f, %1.3f, %1.3f]\n", data->acc_x, data->acc_y, data->acc_z);
        Serial.printf(" Gyr: [%1.3f, %1.3f, %1.3f]\n", data->gyr_x, data->gyr_y, data->gyr_z);
        Serial.printf(" Pos: [%1.3f, %1.3f, %1.3f]\n", data->pos_x, data->pos_y, data->pos_z);
        Serial.printf(" Quat: [%1.3f, %1.3f, %1.3f, %1.3f]\n", data->quat_w, data->quat_x, data->quat_y, data->quat_z);
        Serial.println("\n----------------------------------------");
    }
}

Control::state_t<float> imu_integrator(imu_vicon_t &data, float dt)
{
    Control::state_t<float> state;
    state.setZero();

    state(0) = data.pos_x;
    state(1) = data.pos_y;
    state(2) = data.pos_z;
    state(3) = data.quat_w;
    state(4) = data.quat_x;
    state(5) = data.quat_y;
    state(6) = data.quat_z;

    // Finite diff of position for new velocity estimate
    state(seqN(7, 3)) = (state(seqN(0, 3)) - last_pos) / dt;
    last_pos = state(seqN(0, 3));

    state(10) = data.gyr_x;
    state(11) = data.gyr_y;
    state(12) = data.gyr_z;

    return state;
}

void onPacketReceived(const uint8_t *buffer, size_t bytes_recv)
{
    uint8_t crc8_byte_recv = buffer[bytes_recv - 1];
    uint8_t crc8_byte_comp = crc8(decode_params, (const uint8_t *)buffer, bytes_recv - 1);
    size_t bytes_expected = sizeof(imu_vicon_t) + 1;

    if ((crc8_byte_comp == crc8_byte_recv) && (bytes_recv == bytes_expected))
    {
        memcpy(&data, buffer, sizeof(imu_vicon_t));
        new_imu_vicon = true;
        last_time = time;
        time = micros();
    }
    else
    {
        DEBUG_PRINT("Failed to parse packet with CRC8!");
    }
}

void initFeatherPacket()
{
    Serial3.begin(115200);
    while (!Serial3)
    {
        delay(10);
    }
    DEBUG_PRINT("Started Serial3");

    featherPacketSerial.setStream(&Serial3);
    featherPacketSerial.setPacketHandler(&onPacketReceived);
}

bool receivedFeatherPacket()
{
    return new_imu_vicon;
}

float getFeatherPacket(Filter::input_t<float> *filt_input,
                       Filter::measurement_t<float> *filt_meas)
{
    featherPacketSerial.update();
    if (featherPacketSerial.overflow())
    {
        digitalWrite(LED_PIN, HIGH);
    }

    if (receivedFeatherPacket())
    {
        new_imu_vicon = false;
        imu_vicon_to_filt(data, filt_input, filt_meas);
        float dt = (time - last_time) / 1e6;
        return dt;
    }

    return -1;
}

#endif