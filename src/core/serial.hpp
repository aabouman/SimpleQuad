#pragma once

#include <chrono>
#include <inttypes.h>
#include <string>
#include <stdexcept>
#include <vector>

#include <libserialport.h>

namespace rexlab {

std::vector<std::string> GetPortList();

struct sp_port* InitializeSerialPort(std::string& port, int baudrate);

class LibSerialPortError : public std::runtime_error {
 public:
  LibSerialPortError(std::string arg) : std::runtime_error(std::move(arg)) {}
};

/**
 * @brief Check return codes for libserial and throw errors as necessary
 * 
 * @param result true if no errors are round
 * @throw LibSerialPortError if an error code is read 
 */
bool LibSerialCheck(enum sp_return result);

void HandleLibSerialError(enum sp_return result);



}  // namespace rexlab