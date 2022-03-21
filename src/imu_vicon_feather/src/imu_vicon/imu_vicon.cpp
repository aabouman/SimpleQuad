#include "imu_vicon.hpp"

#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define EIGEN_NO_MALLOC
#include <ArduinoEigenDense.h>
#include <ArduinoEigen/Eigen/Geometry>

using namespace Eigen;

struct imu_vicon_relay
{
    // IMU attributes
    sensors_event_t imu_event;
    Adafruit_BNO055 bno;
    // VICON attributes
    rexlab::Pose<float> vicon_float;
    rexlab::Pose<int16_t> vicon_int16;
    uint8_t buf[POSE_MSG_SIZE];
    bool new_msg;
    // Default to Identity quaternion
    Quaternionf offset_quat;
};

// Global Variables
imu_vicon_relay receiver;

// Declaration of callback function for LoRa
static void onLoRaReceive(int packetSize);

void init_imuViconRelay()
{
    // Setup IMU params
    receiver.bno = Adafruit_BNO055(SENSOR_ID, IMU_ADDRESS, &Wire);
    if (!(receiver.bno.begin()))
    {
        if (Serial)
        {
            Serial.print("\nOoops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
        }
        while (true) ;
    }
    receiver.offset_quat = Quaternionf(1, 0, 0, 0);

    // Setup VICON params
    rexlab::Pose<float> vicon_float;
    rexlab::Pose<int16_t> vicon_int16;
    // Initialize global variable
    receiver.vicon_float = vicon_float;
    receiver.vicon_int16 = vicon_int16;
    receiver.new_msg = false;
    // Setup LoRa Communications
    LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);
    if (!LoRa.begin(915E6))
    {
        if (Serial)
        {
            Serial.println("Starting LoRa failed!");
        }
        while (true);
    }
    // Optimal speed settings
    LoRa.setSpreadingFactor(6);
    LoRa.setSignalBandwidth(500E3);

    LoRa.onReceive(onLoRaReceive);
    LoRa.receive(POSE_MSG_SIZE);
    LoRa.enableCrc();

    return;
}

static void onLoRaReceive(int packetSize)
{
    if (packetSize)
    {
        LoRa.readBytes(receiver.buf, POSE_MSG_SIZE);
        receiver.vicon_int16 = *((rexlab::Pose<int16_t> *)receiver.buf);

        receiver.new_msg = true;
    }
}

bool hasLoRaReceived()
{
    return receiver.new_msg;
}

void ConvertPoseToVicon(const rexlab::Pose<float> &pose, imu_vicon_t *data)
{
    data->pos_x = pose.position_x;
    data->pos_y = pose.position_y;
    data->pos_z = pose.position_z;
    data->quat_w = pose.quaternion_w;
    data->quat_x = pose.quaternion_x;
    data->quat_y = pose.quaternion_y;
    data->quat_z = pose.quaternion_z;
    data->time = static_cast<double>(pose.time_us) / 1e6;
}

void updateVicon(imu_vicon_t *data)
{
    ConvertPoseIntToFloat(receiver.vicon_int16, &(receiver.vicon_float));
    ConvertPoseToVicon(receiver.vicon_float, data);
    receiver.new_msg = false;
}

void updateIMU(imu_vicon_t *data)
{
    receiver.bno.getEvent(&(receiver.imu_event));
    imu::Vector<3> acc = receiver.bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyr = receiver.bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    Vector3f acc_eig(acc.x(), acc.y(), acc.z());
    Vector3f gyr_eig(gyr.x(), gyr.y(), gyr.z());
    acc_eig = receiver.offset_quat * acc_eig;
    gyr_eig = receiver.offset_quat * gyr_eig;

    // Populate translational acceleration
    data->acc_x = acc_eig(0);
    data->acc_y = acc_eig(1);
    data->acc_z = acc_eig(2);
    // Populate rotational velocity
    data->gyr_x = gyr_eig(0);
    data->gyr_y = gyr_eig(1);
    data->gyr_z = gyr_eig(2);
}

void displayCalStatus()
{
    uint8_t system, gyro, accel, mag = 0;
    receiver.bno.getCalibration(&system, &gyro, &accel, &mag);
    if (Serial)
    {
        /* Display the individual values */
        Serial.println("\n-----------Sensor Calibration-----------");
        Serial.printf("\tSys: %d\tGyr: %d\tAcc: %d\tMag: %d", system, gyro, accel, mag);
        Serial.println("\n----------------------------------------");
    }
}

void displaySensorReading()
{
    imu::Vector<3> gyr = receiver.bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> acc = receiver.bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> mag = receiver.bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    if (Serial)
    {
        /* Display the individual values */
        Serial.println("\n-------------Sensor Reading-------------");
        Serial.printf(" Gyr: [%1.3f, %1.3f, %1.3f]\n", gyr.x(), gyr.y(), gyr.z());
        Serial.printf(" Acc: [%1.3f, %1.3f, %1.3f]\n", acc.x(), acc.y(), acc.z());
        Serial.printf(" Mag: [%1.3f, %1.3f, %1.3f]\n", mag.x(), mag.y(), mag.z());
        Serial.println("\n----------------------------------------");
    }
}

void calibrateIMU()
{
    receiver.bno.getEvent(&(receiver.imu_event));
    imu::Vector<3> acc = receiver.bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // Populate translational acceleration averages
    float ave_acc_x = acc.x();
    float ave_acc_y = acc.y();
    float ave_acc_z = acc.z();

    for (int i = 0; i < 1000; i++)
    {
        // Fetch new data
        receiver.bno.getEvent(&(receiver.imu_event));
        acc = receiver.bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        // Rolling average
        ave_acc_x = (ave_acc_x + acc.x()) / 2;
        ave_acc_y = (ave_acc_y + acc.y()) / 2;
        ave_acc_z = (ave_acc_z + acc.z()) / 2;
    }

    Vector3f true_grav(0, 0, -9.81);
    Vector3f meas_grav(ave_acc_x, ave_acc_y, ave_acc_z);

    receiver.offset_quat = Quaternionf::FromTwoVectors(meas_grav, true_grav);
    // Serial.printf("offset_quat: [%f,  %f,  %f,  %f]",
    //               receiver.offset_quat.w(), receiver.offset_quat.x(),
    //               receiver.offset_quat.y(), receiver.offset_quat.z());
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

void constraintCheck(imu_vicon_t *data)
{
    double quat_norm = sqrt(pow(data->quat_w, 2) +
                            pow(data->quat_x, 2) +
                            pow(data->quat_y, 2) +
                            pow(data->quat_z, 2));

    if (abs(quat_norm - 1) > .001)
    {
        Serial.println("Quaternion Constraint Violation!!!");
        Serial.printf(" Quat: [%1.3f, %1.3f, %1.3f, %1.3f]\n", data->quat_w, data->quat_x, data->quat_y, data->quat_z);
    }
}

