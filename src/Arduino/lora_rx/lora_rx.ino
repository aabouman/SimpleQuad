#include <string>

#include <SPI.h>
#include <LoRa.h>

#include "pose.hpp"
#include "arduino_receiver.hpp"

// using Pose = rexlab::Pose<int16_t>;
using Pose = rexlab::PoseMsg;

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define LED_PIN 13
constexpr int MSG_SIZE = sizeof(Pose);

uint8_t buf[MSG_SIZE];

void onReceive(int packetSize) {
    if (packetSize) {
        LoRa.readBytes(buf, MSG_SIZE);
        // Serial.println("LoRa packet received!");
        Serial.write(buf, MSG_SIZE);
    }
}

void setup() {
    Serial.begin(57600);
    while (!Serial) { delay(10); }
    // Serial.println("LoRa Receiver");
    LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);

    if (!LoRa.begin(915E6)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
    LoRa.setSpreadingFactor(6);
    LoRa.setSignalBandwidth(500E3);

    LoRa.onReceive(onReceive);
    LoRa.receive(MSG_SIZE);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    // Serial.println("Hello from LoRa!");
    digitalWrite(LED_PIN, LOW);
    delay(1000);
}