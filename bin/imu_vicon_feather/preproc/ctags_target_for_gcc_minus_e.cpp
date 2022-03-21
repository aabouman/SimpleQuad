# 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino"
# 2 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino" 2
# 3 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino" 2
# 4 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino" 2

# 6 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino" 2

// #define DEBUG




void sendTeensyMessage(imu_vicon_t &data);

imu_vicon_t data = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 };
crc8_params params = { 0xD5, 0x00, 0x00, false, false };
PacketSerial teensyPacketSerial;

bool led_state = (0x0);

// Startup
void setup()
{
    pinMode(13, (0x1));
    digitalWrite(13, led_state);
# 35 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/imu_vicon_feather.ino"
    // Initialize IMU VICON Relay and point to it with global
    init_imuViconRelay();

    calibrateIMU();

    Serial1.begin(115200);
    while (!Serial1)
    {
        delay(10);
    }
    if (Serial)
    {
        Serial.println("Serial1 started");
    }

    teensyPacketSerial.setStream(&Serial1);
}

void loop()
{
    // Limit to 100 Hz
    delay(10);
    teensyPacketSerial.update();

    // Update imu entry
    updateIMU(&data);

    // If LoRa has received update vicon entry and send onto the Teensy
    if (hasLoRaReceived())
    {
        updateVicon(&data);
        // Send info to Teensy
        sendTeensyMessage(data);
    }




}

/*
 * Relay the message over from LoRa/IMU to Jetson
 */
void sendTeensyMessage(imu_vicon_t &data)
{
    // Copy IMU_VICON message into buffer of size sizeof(IMU_VICON)+1
    size_t imu_vicon_size = sizeof(imu_vicon_t);
    uint8_t buffer[imu_vicon_size + 1];

    memcpy(buffer, &data, imu_vicon_size);
    // Compute the CRC8 value of the IMU_VICON message
    uint8_t crc = crc8(params, buffer, imu_vicon_size);
    buffer[imu_vicon_size] = crc;
    // Write IMU_VICON value along with crc8 value
    teensyPacketSerial.send(buffer, imu_vicon_size + 1);

    digitalWrite(13, led_state);
    led_state = !led_state;
}
