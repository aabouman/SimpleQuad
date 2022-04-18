# 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_id_teensy/imu_id_teensy.ino"
# 2 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_id_teensy/imu_id_teensy.ino" 2
# 3 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_id_teensy/imu_id_teensy.ino" 2
# 4 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_id_teensy/imu_id_teensy.ino" 2
# 5 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_id_teensy/imu_id_teensy.ino" 2


// #define DEBUG

struct imu_data_t
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;
};

void sendLaptopMessage(imu_data_t &data);

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
sensors_event_t imu_event;
imu_data_t data;

bool led_state = 0;

// Startup
void setup()
{
    pinMode(13, 1);
    digitalWrite(13, led_state);

    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }
    digitalWrite(13, 1);

    if (!(bno.begin(Adafruit_BNO055::OPERATION_MODE_ACCGYRO)))
    {
        Serial.print("\nOoops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
        while (true);
    }
}

void loop()
{
    bno.getEvent(&imu_event);
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    data.acc_x = acc[0];
    data.acc_y = acc[1];
    data.acc_z = acc[2];
    data.gyr_x = gyr[0];
    data.gyr_y = gyr[1];
    data.gyr_z = gyr[2];
# 68 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_id_teensy/imu_id_teensy.ino"
    sendLaptopMessage(data);

}

void sendLaptopMessage(imu_data_t &data)
{
    // Write IMU_VICON value along with crc8 value
    Serial.write((const uint8_t *) &data, sizeof(imu_data_t));
    Serial.write("End");
    digitalWrite(13, led_state);
    led_state = !led_state;
}
