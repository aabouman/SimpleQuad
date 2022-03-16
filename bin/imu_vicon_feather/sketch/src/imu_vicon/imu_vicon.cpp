#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/src/imu_vicon/imu_vicon.cpp"
#include "imu_vicon.hpp"

#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

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
    imu::Vector<3> gyr = receiver.bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> acc = receiver.bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // Populate translational acceleration
    data->acc_x = acc.x();
    data->acc_y = acc.y();
    data->acc_z = acc.z();
    // Populate rotational velocity
    data->gyr_x = gyr.x();
    data->gyr_y = gyr.y();
    data->gyr_z = gyr.z();
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

bool calibrateIMU()
{
    receiver.bno.setExtCrystalUse(true);
    sensors_event_t event;

    adafruit_bno055_offsets_t calibrationData = {15293, 0, 615, 50, 0, 0, 8192, 512, 0, 0, 512};
    receiver.bno.setSensorOffsets(calibrationData);
    if (Serial)
    {
        Serial.println("Calibration data loaded into BNO055");
    }
    delay(1000);
    if(Serial)
    {
        Serial.println("Checking Sensor Calibration: ");
    }
    while (!receiver.bno.isFullyCalibrated())
    {
        receiver.bno.getEvent(&event);
        displayCalStatus();
        delay(100);
    }
    if(Serial)
    {
        Serial.printf("Calibration status: %d", receiver.bno.isFullyCalibrated());
    }
    return true;
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
