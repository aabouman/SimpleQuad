#include "callbacks.hpp"
#include "NatNetTypes.h"
#include "fmt/core.h"
#include "utils.hpp"
#include <string>

#include <gtest/gtest.h>

TEST(LORATEST, Connection) {
    const char* port = "/dev/ttyACM0";
    int baudrate = 57600;
    rexlab::SerialCallback lora(port, baudrate);

    sRigidBodyData rbdata; 
    std::vector<std::string> ports = rexlab::GetPortList();
    for (auto& port : ports) {
        fmt::print("{}\n", port);
    }
    lora.Open();
    EXPECT_TRUE(lora.IsOpen());
}