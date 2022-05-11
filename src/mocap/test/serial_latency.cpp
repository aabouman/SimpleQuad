#include <libserialport.h>
#include <fmt/core.h>

#include "core/utils.hpp"

int main() {
  struct sp_port* tx;
  std::string rx_name = "/dev/ttyACM0";
  int baudrate = 57600;
  rexlab::LibSerialCheck(sp_get_port_by_name(rx_name.c_str(), &tx));
  rexlab::LibSerialCheck(sp_open(tx, SP_MODE_READ));
  rexlab::LibSerialCheck(sp_set_baudrate(tx, baudrate));
  rexlab::LibSerialCheck(sp_set_bits(tx, 8));
  rexlab::LibSerialCheck(sp_set_parity(tx, SP_PARITY_NONE));
  rexlab::LibSerialCheck(sp_set_stopbits(tx, 1));
  rexlab::LibSerialCheck(sp_set_flowcontrol(tx, SP_FLOWCONTROL_NONE));
  fmt::print("Connected to Receiver\n");
}