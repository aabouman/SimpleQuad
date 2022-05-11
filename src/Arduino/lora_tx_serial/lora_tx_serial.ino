#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define LED_PIN 13
#define MSG_SIZE 5
const int MsgID = 111;

char buf[100];

#include "arduino_receiver.hpp"

rexlab::SerialReceiver<Serial_> receiver(Serial, MsgID);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(57600);
  Serial1.begin(57600);
}

void loop() {
  bool did_receive = receiver.Receive(buf, MSG_SIZE);
  // int bytes_available = Serial.available();
  if (did_receive) {
    // Serial.readBytes(buf, bytes_available);
    Serial.write(buf, MSG_SIZE);
    Serial1.write(buf, MSG_SIZE);
  }
}