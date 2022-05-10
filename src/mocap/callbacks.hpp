#pragma once

#include <NatNetTypes.h>
#include <chrono>
#include <string>

#include "libserialport.h"

namespace rexlab {

class SerialCallback {
public:
  SerialCallback(const std::string &port_name, int baudrate);
  bool Open();
  bool IsOpen() { return is_open_; }
  void Close();
  bool WriteBytes(const char *data, size_t size);
  void SetTimeout(int time_ms);

  template <class Duration> void SetTimeout(Duration time) { timeout_ = time; }

  void operator()(const sRigidBodyData &data) {
    // TODO: convert rigid body data to bytes
    (void)data;
  }

private:
  std::string port_name_;
  int baud_rate_;
  struct sp_port *port_;
  bool is_open_ = false;
  std::chrono::milliseconds timeout_ = std::chrono::milliseconds(100);
};

class PrintCallback {
public:
  PrintCallback() = default;
  PrintCallback(FILE* io) : io_(io) {}
  PrintCallback(const std::string &filename) : is_file_(true) {
    io_ = fopen(filename.c_str(), "w");
  }

  ~PrintCallback() {
    if (is_file_) {
      fclose(io_);
    }
  }

 void operator()(const sRigidBodyData& data) {
    bool bTrackingValid = data.params & 0x01;
    printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n",
           data.ID, data.MeanError,
           bTrackingValid);
    printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
    printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
           data.x,  data.y,
           data.z,  data.qx,
           data.qy, data.qz,
           data.qw);
  }

private: 
    FILE* io_ = stdout;
    bool is_file_ = false;
};

}  // namespace rexlab