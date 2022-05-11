#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define LED_PIN 13
#define MSG_SIZE 6
const int MsgID = 120;  // 'xx


// #include "arduino_receiver.hpp"

// rexlab::SerialReceiver<Serial_> receiver(Serial, MsgID);
#include "serial_utils.hpp"

char buf[200];
char msg[MSG_SIZE];
int bytes_read = 0;  // number of message bytes read so far
int pos = 0;  // position in buffer
int total_bytes_in_buffer = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(57600);
  Serial1.begin(57600);
}

void loop() {
  // // Read all available bytes into buffer
  int bytes_available = Serial.available();
  int bytes_received = Serial.readBytes(buf, bytes_available);

  // for (int i = 0; i < bytes_received; ++i) {
  //   // If starting a new message, must start with message ID
  //   if (pos == 0) {
  //     if (buf[i] == MsgID) {
  //       msg[pos] = buf[i];
  //       ++pos;
  //     }
  //   // If in the middle of a message, accept all bytes
  //   } else if (pos < MSG_SIZE) {
  //     msg[pos] = buf[i];
  //     ++pos;
  //     // Message completed
  //     if (pos == MSG_SIZE) {
  //       pos = 0;
  //       Serial1.write(msg, MSG_SIZE);
  //     }
  //   } 
  // }

  if (bytes_received) {
    int msg_id = MsgID;
    int start_index;
    uint8_t byte;
    for (start_index = 0; start_index < bytes_received; ++start_index) {
      byte  = buf[start_index];
      if (byte == msg_id) {
        break;
      }
    }

    // How many bytes of the message did we receive?
    int received_length = bytes_received - start_index;  

    // Shift the data over to the start of the buffer
    //   discards all bytes before the MsgID
    if (start_index > 0) {
      for (int i = 0; i < received_length; ++i) {
        buf[i] = buf[i + start_index];
      }
    }

    Serial1.write(buf, received_length);
    Serial.print("Location of msgID: ");
    Serial.print(start_index);
    Serial.print(". Received length: ");
    Serial.println(received_length);
  }
  // int bytes_received = rexlab::ReadMessageWithID(Serial, buf, 100, MSG_SIZE, MsgID);
  // delay(1000);
  // bool did_receive = bytes_received > 0;
  // if (did_receive) {
  //   // Serial.readBytes(buf, bytes_available);
  //   Serial1.write(buf, bytes_received);
  // }
}