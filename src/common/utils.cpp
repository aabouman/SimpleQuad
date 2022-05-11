#include "utils.hpp"

#include <iostream>
// #include <fmt/core.h>
// #include <fmt/chrono.h>

namespace rexlab {

// int VerifyRead(char* buf, int len, uint8_t msg_id) {
//   // Search for start of the message
//   int start_index;
//   uint8_t byte;
//   for (start_index = 0; start_index < len; ++start_index) {
//     byte  = buf[start_index];
//     if (byte == msg_id) {
//       break;
//     }
//   }

//   if (start_index > 0) {
//     // How many bytes of the message did we receive?
//     int received_length = len - start_index;  

//     // Shift the data over to the start of the buffer
//     for (int i = 0; i < received_length; ++i) {
//       buf[i] = buf[i + start_index];
//     }
//   }
//   return start_index;
// }

void RatePrinter::Init() {
  time_start_ = std::chrono::high_resolution_clock::now();
  count_ = 0;
}

void RatePrinter::Enable() { is_enabled_ = true; }
void RatePrinter::Disable() { is_enabled_ = false; }
bool RatePrinter::IsEnabled() { return is_enabled_; } 
void RatePrinter::SetFrequency(float frequency) { 
  period_ = std::chrono::duration<double, std::ratio<1>>(1.0 / frequency);
}
void RatePrinter::Print() {
  if (IsEnabled()) {
    std::chrono::duration<double, std::ratio<1>> t_elapsed = std::chrono::high_resolution_clock::now() - time_start_;
    if (t_elapsed > period_) {
      double average_rate = count_ / t_elapsed.count();

      std::cout << "Average rate: " << average_rate << " Hz" << std::endl;
      count_ = 0;
      time_start_ = std::chrono::high_resolution_clock::now();
    }
    ++count_;
  }
}
 
std::string MakeTcpAddress(const std::string& addr) {
  std::string tcp_addr;
  if (addr.rfind("tcp://") == 0) {
    tcp_addr = addr;
  } else {
    tcp_addr = "tcp://" + addr;
  }
  return tcp_addr;
}

TcpAddress::TcpAddress() : addr_(MakeTcpAddress("localhost")) {}
TcpAddress::TcpAddress(std::string addr) : addr_(MakeTcpAddress(std::move(addr))) {}
TcpAddress::TcpAddress(std::string addr, int port) 
    : addr_(MakeTcpAddress(addr)), port_(std::move(port)) {}
TcpAddress::TcpAddress(TcpAddress&& other) : addr_(other.addr_), port_(other.port_) {}

std::string TcpAddress::ToString() {
  std::string port_string;
  if (port_ == kAnyPort) {
    port_string = ":*";
  } else {
    port_string = ":" + std::to_string(port_);
  }
  return addr_ + port_string;
}

}  // namespace rexlab