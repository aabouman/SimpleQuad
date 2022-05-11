#include <chrono>
#include <vector>
#include <unistd.h>

#include <libserialport.h>
#include <fmt/core.h>
#include <fmt/chrono.h>

#include "common/utils.hpp"
#include "utils/serial.hpp"

struct MyMsg {
  float x;
};
constexpr int MSG_SIZE = sizeof(MyMsg);

int main() {
  // Open Serial ports
  std::string tx_name = "/dev/ttyACM0";
  int baudrate = 57600;
  struct sp_port* tx = rexlab::InitializeSerialPort(tx_name, baudrate);
  fmt::print("Connected to Transmitter\n");

  std::string rx_name = "/dev/ttyACM1";
  struct sp_port* rx = rexlab::InitializeSerialPort(rx_name, baudrate);
  fmt::print("Connected to Receiver\n");

  // Send and receive floats
  const int nsamples = 100;
  char buf[MSG_SIZE];
  MyMsg msg;
  msg.x = 0.0;
  std::vector<std::pair<double, float>> datasent;
  std::vector<std::pair<double, float>> datarecv;
  datasent.reserve(nsamples);
  datarecv.reserve(nsamples);
  auto tstart = std::chrono::high_resolution_clock::now();
  using fmillisecond = std::chrono::duration<float, std::milli>;

  for (int i = 0; i < nsamples; ++i) {
    float x_sent = i * 0.001;
    msg.x = x_sent; 

    // Send data
    memcpy(buf, &msg, MSG_SIZE);
    auto tsend = std::chrono::duration_cast<fmillisecond>(
        std::chrono::high_resolution_clock::now() - tstart);
    enum sp_return bytes_sent = sp_blocking_write(tx, buf, MSG_SIZE, 100);

    // Receive
    int bytes_received = sp_blocking_read(rx, buf, MSG_SIZE, 100);
    auto trecv = std::chrono::duration_cast<fmillisecond>(
        std::chrono::high_resolution_clock::now() - tstart);

    // Record info 
    rexlab::HandleLibSerialError(bytes_sent);
    fmt::print("Sample {}\n", i);
    fmt::print("  Sent {} bytes, x = {}\n", (int)bytes_sent, msg.x);
    memcpy(&msg, buf, MSG_SIZE);
    fmt::print("  Received {} bytes, x = {}\n", bytes_received, msg.x);
    fmt::print("  Latency {} ms\n", trecv - tsend);

    datasent.emplace_back(std::make_pair(tsend.count(), x_sent));
    datarecv.emplace_back(std::make_pair(trecv.count(), msg.x));
    usleep(10 * 1000);
  }
  
  // Write data to file
  FILE* outfile = fopen("src/mocap/test/serial_chain_latency_test_out.txt", "w");
  FILE* infile = fopen("src/mocap/test/serial_chain_latency_test_in.txt", "w");
  for (auto& pair : datasent) {
    fmt::print(outfile, "{:0.4f}, {:0.5f}\n", pair.first, pair.second); 
  }
  for (auto& pair : datarecv) {
    fmt::print(infile, "{:0.4f}, {:0.5f}\n", pair.first, pair.second); 
  }
  fclose(outfile);
  fclose(infile);
  return 0;
}