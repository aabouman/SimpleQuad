#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_relay.cpp"
#include "imu_vicon_relay.hpp"

// For the IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
// For the LoRa
#include <SPI.h>
#include <LoRa.h>
#include <math.h>

#include "pose.hpp"

// Size of lora buffer
constexpr int POSE_MSG_SIZE = sizeof(rexlab::Pose<int16_t>);

struct LoRaViconReceiver
{
    uint8_t lora_buffer[POSE_MSG_SIZE];

    rexlab::Pose<int16_t> vicon_int16;
    rexlab::Pose<float> vicon_float;

    bool new_msg;
};

struct ImuVicon
{
    // IMU data
    sensors_event_t imu_event;
    Adafruit_BNO055 imu;
    // Vicon Data
    LoRaViconReceiver vicon;
};

// Global Variables
sensors_event_t global_imu_event;
Adafruit_BNO055 global_imu;
LoRaViconReceiver global_vicon;
ImuVicon global_receiver = {global_imu_event, global_imu, global_vicon};

bool initializeImuVicon()
{
    global_receiver.imu = Adafruit_BNO055(SENSOR_ID, IMU_ADDRESS, &Wire)

        // rexlab::Pose<float> vicon_float;
        // rexlab::Pose<int16_t> vicon_int16;

        // Initialize global variable
        // global_receiver.vicon.vicon_float = vicon_float;
        // global_receiver.vicon.vicon_int16 = vicon_int16;
        // global_receiver.vicon.buf = POSE_MSG_SIZE;

    global_receiver.vicon.new_msg = false;

    // Setup LoRa Communications
    LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);
    if (!LoRa.begin(915E6))
    {
        Serial.println("Starting LoRa failed!");
        while (1){}
    }

    // Optimal speed settings
    LoRa.setSpreadingFactor(6);
    LoRa.setSignalBandwidth(500E3);

    LoRa.onReceive(onLoRaReceive);
    LoRa.receive(global_receiver.vicon.msg_size);
    LoRa.enableCrc();

    return true;
}

void ConvertPoseToVicon(const rexlab::Pose<float> &pose, IMU_VICON *imu_vicon)
{
    imu_vicon->pos_x = pose.position_x;
    imu_vicon->pos_y = pose.position_y;
    imu_vicon->pos_z = pose.position_z;
    imu_vicon->quat_w = pose.quaternion_w;
    imu_vicon->quat_x = pose.quaternion_x;
    imu_vicon->quat_y = pose.quaternion_y;
    imu_vicon->quat_z = pose.quaternion_z;
    imu_vicon->time = static_cast<double>(pose.time_us) / 1e6;
}

void updateVicon(IMU_VICON &imu_vicon)
{
    ConvertPoseIntToFloat(global_receiver.vicon_int16, &global_receiver.vicon_float);
    ConvertPoseToVicon(global_receiver.vicon_float, &imu_vicon);
    global_receiver.new_msg = false;
}

void onLoRaReceive(int packetSize)
{
    if (packetSize)
    {
        LoRa.readBytes(global_receiver.buf, global_receiver.msg_size);
        // Serial.write(global_receiver.buf, global_receiver.msg_size);
        global_receiver.vicon_int16 = *((rexlab::Pose<int16_t> *)global_receiver.buf);

        global_receiver.new_msg = true;
    }
}

bool hasLoRaReceived()
{
    return global_receiver.new_msg;
}


void updateIMU(Adafruit_BNO055 &bno, IMU_VICON &imu_vicon)
{
    bno.getEvent(&imu_event);

    imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // Populate translational acceleration
    imu_vicon.acc_x = acc.x();
    imu_vicon.acc_y = acc.y();
    imu_vicon.acc_z = acc.z();
    // Populate rotational velocity
    imu_vicon.gyr_x = gyr.x();
    imu_vicon.gyr_y = gyr.y();
    imu_vicon.gyr_z = gyr.z();
}

void displayCalStatus(Adafruit_BNO055 &bno)
{
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    /* Display the individual values */
    Serial.println("\n-----------Sensor Calibration-----------");
    Serial.printf("\tSys: %d\tGyr: %d\tAcc: %d\tMag: %d", system, gyro, accel, mag);
    Serial.println("\n----------------------------------------");
}

void displaySensorReading(Adafruit_BNO055 &bno)
{
    imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    /* Display the individual values */
    Serial.println("\n-------------Sensor Reading-------------");
    Serial.printf(" Gyr: [%1.3f, %1.3f, %1.3f]\n", gyr.x(), gyr.y(), gyr.z());
    Serial.printf(" Acc: [%1.3f, %1.3f, %1.3f]\n", acc.x(), acc.y(), acc.z());
    Serial.printf(" Mag: [%1.3f, %1.3f, %1.3f]\n", mag.x(), mag.y(), mag.z());
    Serial.println("\n----------------------------------------");
}

bool calibrateIMU(Adafruit_BNO055 &bno)
{
    bno.setExtCrystalUse(true);
    sensors_event_t event;

    adafruit_bno055_offsets_t calibrationData = {15293, 0, 615, 50, 0, 0, 8192, 512, 0, 0, 512};
    bno.setSensorOffsets(calibrationData);
    Serial.println("Calibration data loaded into BNO055");
    delay(1000);

    Serial.println("Checking Sensor Calibration: ");
    while (!bno.isFullyCalibrated())
    {
        bno.getEvent(&event);
        displayCalStatus(bno);
        delay(100);
    }
    Serial.printf("Calibration status: %d", bno.isFullyCalibrated());

    return true;
}

void displayImuVicon(IMU_VICON &imu_vicon)
{
    /* Display the individual values */
    Serial.println("\n-------------Sensor Reading-------------");
    Serial.printf(" Acc: [%1.3f, %1.3f, %1.3f]\n", imu_vicon.acc_x, imu_vicon.acc_y, imu_vicon.acc_z);
    Serial.printf(" Gyr: [%1.3f, %1.3f, %1.3f]\n", imu_vicon.gyr_x, imu_vicon.gyr_y, imu_vicon.gyr_z);
    Serial.printf(" Pos: [%1.3f, %1.3f, %1.3f]\n", imu_vicon.pos_x, imu_vicon.pos_y, imu_vicon.pos_z);
    Serial.printf(" Quat: [%1.3f, %1.3f, %1.3f, %1.3f]\n", imu_vicon.quat_w, imu_vicon.quat_x, imu_vicon.quat_y, imu_vicon.quat_z);
    Serial.println("\n----------------------------------------");
}

void constraintCheck(IMU_VICON &imu_vicon)
{
    double quat_norm = sqrt(pow(imu_vicon.quat_w, 2) +
                            pow(imu_vicon.quat_x, 2) +
                            pow(imu_vicon.quat_y, 2) +
                            pow(imu_vicon.quat_z, 2));

    if (abs(quat_norm - 1) > .001)
    {
        Serial.println("Quaternion Constraint Violation!!!");
        Serial.printf(" Quat: [%1.3f, %1.3f, %1.3f, %1.3f]\n", imu_vicon.quat_w, imu_vicon.quat_x, imu_vicon.quat_y, imu_vicon.quat_z);
    }
}