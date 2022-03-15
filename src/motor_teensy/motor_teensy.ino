#include <Arduino.h>
#include <PacketSerial.h>
#include <crc8.h>
// #include "src/control/lqr.hpp"
#include "src/kalman/kalman.hpp"

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define LED_PIN 13

void onPacketReceived(const uint8_t *buffer, size_t bytes_recv);

imu_vicon data = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};

crc8_params params = DEFAULT_CRC8_PARAMS;
PacketSerial featherPacketSerial;

// Startup
void setup()
{
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }
    Serial3.begin(115200);
    while (!Serial3)
    {
        delay(10);
    }
    Serial.println("Started Serial3");

    // featherPacketSerial.setStream(&Serial3);
    // featherPacketSerial.setPacketHandler(&onPacketReceived);
}

void loop()
{
    delay(10);
    // featherPacketSerial.update();

    // quad_state_t<float> x

    // input_t<float> u = get_control();
    // Serial.printf("u: [%.4f,  %.4f,  %.4f,  %.4f]\n" u(0));

    // if (featherPacketSerial.overflow())
    // {
    //     digitalWrite(LED_PIN, HIGH);
    // }
}
/*
 * Relay the message over from LoRa/IMU to Jetson
 */
void onPacketReceived(const uint8_t *buffer, size_t bytes_recv)
{

    uint8_t crc8_byte_recv = buffer[bytes_recv - 1];
    uint8_t crc8_byte_comp = crc8(params, (const uint8_t *) buffer, bytes_recv - 1);

    if (crc8_byte_comp == crc8_byte_recv)
    {
        memcpy(&data, &buffer, bytes_recv - 1);
        Serial.print(data.quat_w);
        Serial.println();
    }
    else
    {
        Serial.println("Failed to parse packet with CRC8!");
    }
}
