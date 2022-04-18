#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define LED_PIN 13
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

bool led_state = LOW;

// Startup
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, led_state);

    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }
    digitalWrite(LED_PIN, HIGH);

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

#ifdef DEBUG
    Serial.println("");
    Serial.println("Acceleration");
    Serial.printf("\t[%+.4f, %+.4f, %+.4f]\n", data.acc_x, data.acc_y, data.acc_z);
    Serial.println("Angular Rate");
    Serial.printf("\t[%+.4f, %+.4f, %+.4f]\n", data.gyr_x, data.gyr_y, data.gyr_z);
#endif

#ifndef DEBUG
    sendLaptopMessage(data);
#endif
}

void sendLaptopMessage(imu_data_t &data)
{
    // Write IMU_VICON value along with crc8 value
    Serial.write((const uint8_t *) &data, sizeof(imu_data_t));
    Serial.write("End");
    digitalWrite(LED_PIN, led_state);
    led_state = !led_state;
}
