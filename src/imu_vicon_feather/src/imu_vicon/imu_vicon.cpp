#include <Arduino.h>
#include <math.h>
#include <LoRa.h>
#include "imu_vicon.hpp"

rexlab::Pose<int16_t> msg_global;
rexlab::Pose<float> pose_global;
uint8_t *buf = reinterpret_cast<uint8_t *>(&msg_global);

// ImuViconRelay::ImuViconRelay(){}

ImuViconRelay::ImuViconRelay(int32_t imu_sensor_id,
                             uint8_t imu_address,
                             TwoWire *imu_wire,
                             uint8_t lora_cs,
                             uint8_t lora_rst,
                             uint8_t lora_int)
{
    this->_bno = Adafruit_BNO055(imu_sensor_id, imu_address, imu_wire);
    if (!(this->_bno.begin()))
    {
        while (true)
        {
            Serial.print("\nOoops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
        }
    }

    // Setup LoRa Communications
    LoRa.setPins(lora_cs, lora_rst, lora_int);
    if (!LoRa.begin(RF95_FREQ))
    {
        Serial.println("Starting LoRa failed!");
        while (true);
    }
    Serial.println("LoRa initialized");

    // Optimal speed settings
    LoRa.setSpreadingFactor(6);
    LoRa.setSignalBandwidth(500E3);
    LoRa.enableCrc();

    this->_new_vicon = false;

    // LoRa.onReceive(this->onLoRaReceive);
    LoRa.receive(POSE_MSG_SIZE);
}

ImuViconRelay::~ImuViconRelay()
{
}

static void ConvertPoseToVicon(const rexlab::Pose<float> &pose, IMU_VICON *imu_vicon)
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

bool ImuViconRelay::hasReceived()
{
    int packetSize = LoRa.parsePacket(POSE_MSG_SIZE);
    Serial.printf("Size of message: %d bytes, Expected: %d bytes\n",
                  packetSize, POSE_MSG_SIZE);

    if (packetSize == POSE_MSG_SIZE)
    {
        LoRa.readBytes(this->_lora_buffer, POSE_MSG_SIZE);
        this->_vicon_int16 = *((rexlab::Pose<int16_t> *) this->_lora_buffer);
        this->_new_vicon = true;
    }

    return (this->_new_vicon);
}

void ImuViconRelay::updateVicon(IMU_VICON * imu_vicon)
{
    ConvertPoseIntToFloat(this->_vicon_int16, &this->_vicon_float);
    ConvertPoseToVicon(this->_vicon_float, imu_vicon);
    this->_new_vicon = false;

    // displayImuVicon(imu_vicon);
}

void ImuViconRelay::updateImu(IMU_VICON &imu_vicon)
{
    this->_bno.getEvent(&(this->_imu_event));
    imu::Vector<3> gyr = this->_bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> acc = this->_bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    // Populate translational acceleration
    imu_vicon.acc_x = acc.x();
    imu_vicon.acc_y = acc.y();
    imu_vicon.acc_z = acc.z();

    // Populate rotational velocity
    imu_vicon.gyr_x = gyr.x();
    imu_vicon.gyr_y = gyr.y();
    imu_vicon.gyr_z = gyr.z();
    return;
}

static void displayCalStatus(Adafruit_BNO055 &bno)
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

bool ImuViconRelay::calibrateImu()
{
    this->_bno.setExtCrystalUse(true);

    adafruit_bno055_offsets_t calibrationData = {15293, 0, 615, 50, 0, 0, 8192, 512, 0, 0, 512};
    this->_bno.setSensorOffsets(calibrationData);
    Serial.println("Calibration data loaded into BNO055");
    delay(1000);

    Serial.println("Checking Sensor Calibration: ");
    while (!this->_bno.isFullyCalibrated())
    {
        this->_bno.getEvent(&(this->_imu_event));
        displayCalStatus(this->_bno);
        delay(100);
    }
    Serial.printf("Calibration status: %d", this->_bno.isFullyCalibrated());

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

bool constraintCheck(IMU_VICON &imu_vicon)
{
    double quat_norm = sqrt(pow(imu_vicon.quat_w, 2) +
                            pow(imu_vicon.quat_x, 2) +
                            pow(imu_vicon.quat_y, 2) +
                            pow(imu_vicon.quat_z, 2));
    if (abs(quat_norm - 1) > .001)
    {
        // Serial.println("Quaternion Constraint Violation!!!");
        // Serial.printf(" Quat: [%1.3f, %1.3f, %1.3f, %1.3f]\n", imu_vicon.quat_w, imu_vicon.quat_x, imu_vicon.quat_y, imu_vicon.quat_z);
        return false;
    }
    return true;
}