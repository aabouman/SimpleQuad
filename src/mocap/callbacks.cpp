#include "callbacks.hpp"
#include <string.h>

#include "fmt/core.h"
#include "common/utils.hpp"
#include "utils/serial.hpp"

namespace rexlab {

void RigidBodyToBytes(char* buf, const sRigidBodyData& data) {
    PoseMsg pose = {data.x, data.y, data.z, data.qw, data.qx, data.qy, data.qz};
    PoseToBytes(buf, pose);
}

SerialCallback::SerialCallback(const std::string &port_name, int baud_rate)
    : port_name_(port_name),
      baud_rate_(baud_rate) {}
  
SerialCallback::~SerialCallback() {
  this->Close();
}

bool SerialCallback::Open() {

  try {
    struct sp_port* port;
    std::string port_name = port_name_;
    int baud_rate = baud_rate_;
    fmt::print("Looking for port {}\n", port_name);
    LibSerialCheck(sp_get_port_by_name(port_name.c_str(), &port));

    fmt::print("Port name: {}\n", sp_get_port_name(port));
    fmt::print("Port description: {}\n", sp_get_port_description(port));

    fmt::print("Opening Port.\n");
    enum sp_return result = sp_open(port, SP_MODE_READ_WRITE);
    if (result != SP_OK) {
        fmt::print("Couldn't open serial port\n");
        char* err_msg = sp_last_error_message();
        fmt::print("Got error: {}\n", err_msg);
        sp_free_error_message(err_msg);
    }

    fmt::print("Setting port to baudrate {}\n", baud_rate);
    LibSerialCheck(sp_set_baudrate(port, baud_rate));
    LibSerialCheck(sp_set_bits(port, 8));
    LibSerialCheck(sp_set_parity(port, SP_PARITY_NONE));
    LibSerialCheck(sp_set_stopbits(port, 1));
    LibSerialCheck(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));

    port_ = port;
    is_open_ = true;
    return true;
  } catch(const LibSerialPortError& e) {
    fmt::print("Error Calling libserialport\n");
    char* err_msg = sp_last_error_message();
    fmt::print("Got error: {}\n", err_msg);
    sp_free_error_message(err_msg);
}
  return false;
}

void SerialCallback::Close() {
  if (IsOpen()) {
    fmt::print("Closing port...\n");
    sp_close(port_);
    fmt::print("Port closed!\n");
    sp_free_port(port_);
    is_open_ = false;
  }
}

void SerialCallback::SetTimeout(int time_ms) {
  timeout_ = std::chrono::milliseconds(time_ms);
}

void SerialCallback::operator()(const sRigidBodyData &data) {
    RigidBodyToBytes(buf_, data);
    WriteBytes(buf_, sizeof(PoseMsg) + 1);
}

int SerialCallback::WriteBytes(const char* data, size_t size) {
  // fmt::print("Writing to serial...\n");
  if (IsOpen()) {
    // Check for input from the serial port and print to stdout
    if (check_for_input_ && (sp_input_waiting(port_) > 0)) {
      int input_bytes = sp_input_waiting(port_);
      fmt::print("Got message of length {} from Feather: ", input_bytes);
      char* tempbuf = (char*) malloc(input_bytes);
      sp_blocking_read(port_, tempbuf, std::min(1000, input_bytes), 100);

      // Convert char array to valid string
      std::string message = "";
      for (int i = 0; i < input_bytes; ++i) {
        message += tempbuf[i];
      }
      fmt::print("{}", message);
    }

    // Write the pose data to the serial port
    // fmt::print("Writing data\n");
    int bytes = sp_blocking_write(port_, data, size, timeout_.count());
    // fmt::print("Finished writing data. Wrote {} bytes\n", bytes);
    // sp_drain(port_);  // make sure the transmission is complete before continuing
    // fmt::print("Finished drain\n");
    return bytes;
  } else {
    fmt::print("WARNING: Trying to write to a closed port!\n");
  }
  return 0;
}

// void ZMQCallback::WriteBytes(const char* data, size_t size) {
//   // fmt::print("Sending ZMQ message...\n");
//   socket_.send(zmq::message_t(data, size), zmq::send_flags::none);
// }



}  // namespace rexlab_