#pragma once

#include <chrono>
#include <inttypes.h>
#include <string>
#include <stdexcept>
#include <vector>


namespace rexlab {

/**
 * @brief Make sure the message starts with the byte `msg_id`.
 * 
 * If the message does not start with a byte matching `msg_id`, find the byte with
 * `msg_id`, shift the data, and report how many extra bytes need to be read to 
 * complete the message.
 * 
 * TODO: Handle the case when the message ID is not found at all.
 * 
 * @param buf Data buffer
 * @param len Length of data buffer
 * @param msg_id Expected first byte
 * @return int Number of extra bytes to read. If 0, the message is valid.
 */
int VerifyRead(char* buf, int len, uint8_t msg_id);

class RatePrinter {
 public:
  void Init();
  void SetFrequency(float frequency);
  bool IsEnabled();
  void Enable();
  void Disable();
  void Print();
 private:
  std::chrono::system_clock::time_point time_start_;
  std::chrono::duration<double, std::ratio<1>> period_ = std::chrono::seconds(1);
  bool is_enabled_;
  int count_ = 0;
};

std::string MakeTcpAddress(const std::string& addr);

class TcpAddress {
public:
  TcpAddress();
  TcpAddress(std::string addr_, int port);
  TcpAddress(std::string addr_);
  TcpAddress(TcpAddress&& other);
  std::string ToString();
  static constexpr int kAnyPort = -1; 

 private:
  std::string addr_;
  int port_ = kAnyPort;
};

}