#include "serial.hpp"

#include <fmt/core.h>

namespace rexlab {

std::vector<std::string> GetPortList() {
  std::vector<std::string> ports;

  struct sp_port** port_list;
  enum sp_return result = sp_list_ports(&port_list);
  if (result != SP_OK) {
    fmt::print("Couldn't get the list of ports\n");
  }

  int i;
  for (i = 0; port_list[i] != NULL; i++) {
    struct sp_port* port = port_list[i];
    ports.emplace_back(sp_get_port_name(port));
  }
  return ports;
}

struct sp_port* InitializeSerialPort(std::string& portpath, int baudrate) {
  struct sp_port* port;

  rexlab::HandleLibSerialError(sp_get_port_by_name(portpath.c_str(), &port));
  fmt::print("Port name: {}\n", sp_get_port_name(port));
  fmt::print("Port description: {}\n", sp_get_port_description(port));
  rexlab::HandleLibSerialError(sp_open(port, SP_MODE_READ_WRITE));
  rexlab::HandleLibSerialError(sp_set_baudrate(port, baudrate));
  rexlab::HandleLibSerialError(sp_set_bits(port, 8));
  rexlab::HandleLibSerialError(sp_set_parity(port, SP_PARITY_NONE));
  rexlab::HandleLibSerialError(sp_set_stopbits(port, 1));
  rexlab::HandleLibSerialError(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));
  return port;
}


/* Helper function for error handling. */
bool LibSerialCheck(enum sp_return result) {
  char* err_msg;
  switch (result)
  {
  case SP_ERR_ARG:
    throw(LibSerialPortError("Error: Invalid argument."));
    break;

  case SP_ERR_FAIL:
    err_msg = sp_last_error_message();
    throw(LibSerialPortError("Error: Failed: " + std::string(err_msg)));
    sp_free_error_message(err_msg);
    break;

  case SP_ERR_SUPP:
    throw(LibSerialPortError("Error: Not Supported."));
    break;

  case SP_ERR_MEM:
    throw(LibSerialPortError("Error: Couldn't allocate memory."));
    break;

  case SP_OK:
  default:
    return true;
  }
}

void HandleLibSerialError(enum sp_return result) {
  try {
    LibSerialCheck(result);
  } catch (LibSerialPortError& e) {
    fmt::print("Error Calling libserialport\n");
    char* err_msg = sp_last_error_message();
    fmt::print("Got error: {}\n", err_msg);
    sp_free_error_message(err_msg);
  }
}

}