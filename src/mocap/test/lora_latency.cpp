#include <bits/types/FILE.h>
#include <string>
#include <gtest/gtest.h>
#include <unistd.h>

#include <libserialport.h>

#include "callbacks.hpp"
#include "NatNetTypes.h"
#include "fmt/core.h"
#include "utils.hpp"


int main() {
    std::vector<std::string> ports = rexlab::GetPortList();
    fmt::print("Available Ports:\n");
    for (auto& port : ports) {
        fmt::print("  {}\n", port);
    }

    struct sp_port* rx;
    std::string rx_name = "/dev/ttyACM1";
    int baudrate = 57600;
    rexlab::LibSerialCheck(sp_get_port_by_name(rx_name.c_str(), &rx));
    rexlab::LibSerialCheck(sp_open(rx, SP_MODE_READ));
    rexlab::LibSerialCheck(sp_set_baudrate(rx, baudrate));
    rexlab::LibSerialCheck(sp_set_bits(rx, 8));
    rexlab::LibSerialCheck(sp_set_parity(rx, SP_PARITY_NONE));
    rexlab::LibSerialCheck(sp_set_stopbits(rx, 1));
    rexlab::LibSerialCheck(sp_set_flowcontrol(rx, SP_FLOWCONTROL_NONE));
    fmt::print("Connected to Receiver\n");

    const char* port = "/dev/ttyACM0";
    rexlab::SerialCallback lora(port, baudrate);
    lora.SetTimeout(200);

    sRigidBodyData rbdata; 
    lora.Open();
    fmt::print("Connect to Transmitter\n");

    fmt::print("Starting Loop...\n");
    while (true) {
        int input_bytes = sp_input_waiting(rx);
        if (input_bytes > 0) {
            char* tempbuf = (char*) malloc(input_bytes);
            sp_blocking_read(rx, tempbuf, input_bytes, 100);
            std::string message = "";
            for (int i = 0; i < input_bytes; ++i) {
                message += tempbuf[i];
            }
            fmt::print("Received {} bytes\n", input_bytes);
        }
        rbdata.x += 0.001;
        lora(rbdata);
    }

    // for (int i = 0; i < 1000; ++i) {
    //     fmt::print("Sending message {}\n", i);
    //     lora(rbdata);
    //     // usleep(1000);
    // }
    // lora.Close();
    sp_close(rx);

    return 0;
}