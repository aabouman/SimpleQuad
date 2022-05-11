#include <chrono>
#include <cstdint>
#include <vector>
#include <unistd.h>

#include <libserialport.h>
#include <fmt/core.h>
#include <fmt/chrono.h>

#include "common/utils.hpp"
#include "utils/serial.hpp"

struct MyMsg {
  static constexpr uint8_t MsgID() { return 120; }
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
  char buf[MSG_SIZE+1];
  char recv[100];
  MyMsg msg;
  msg.x = 0.0;
  std::vector<std::pair<double, float>> datasent;
  std::vector<std::pair<double, float>> datarecv;
  datasent.reserve(nsamples);
  datarecv.reserve(nsamples);
  auto tstart = std::chrono::high_resolution_clock::now();
  using fmillisecond = std::chrono::duration<float, std::milli>;


  // Sending info
  fmt::print("\nSending...\n");
  // memcpy(buf+1, &msg, MSG_SIZE);
  buf[0] = MyMsg::MsgID();
  for (int i = 0; i < MSG_SIZE; ++i) {
    buf[i+1] = i + 'a';
  }
  enum sp_return bytes_sent = sp_blocking_write(tx, buf, MSG_SIZE+1, 100);
  fmt::print("  Sent {} bytes\n", (int)bytes_sent);
  fmt::print("  Message: [");
  for (int i = 0; i < (int)bytes_sent; ++i) {
    fmt::print("{}", buf[i]);
  }
  fmt::print("]\n");

  int bytes_available_tx = 0;
  bytes_available_tx = sp_blocking_read(tx, recv, 100, 10);
  fmt::print("  Message from Tx: ");
  for (int i = 0; i < bytes_available_tx; ++i) {
    fmt::print("{}", recv[i]);
  }
  fmt::print("\n");

  // Receiving
  fmt::print("\nReceiving...\n");
  int bytes_received = sp_blocking_read(rx, recv, 100, 10);
  fmt::print("{} bytes received\n", bytes_received);
  fmt::print("  Message from Rx: [");
  for (int i = 0; i < bytes_received; ++i) {
    fmt::print("{}", recv[i]);
  }
  fmt::print("]\n");

    
  return 0;

  for (int i = 0; i < nsamples; ++i) {
    float x_sent = i * 0.001;
    msg.x = x_sent; 

    // Send data
    memcpy(buf+1, &msg, MSG_SIZE);
    buf[0] = MyMsg::MsgID();
    auto tsend = std::chrono::duration_cast<fmillisecond>(
        std::chrono::high_resolution_clock::now() - tstart);
    enum sp_return bytes_sent = sp_blocking_write(tx, buf, MSG_SIZE+1, 100);

    // Receive
    int bytes_received = sp_blocking_read(rx, buf, MSG_SIZE+1, 100);
    auto trecv = std::chrono::duration_cast<fmillisecond>(
        std::chrono::high_resolution_clock::now() - tstart);

    // Record info 
    rexlab::HandleLibSerialError(bytes_sent);
    fmt::print("Sample {}\n", i);
    fmt::print("  Sent {} bytes, x = {}\n", (int)bytes_sent, msg.x);
    memcpy(&msg, buf+1, MSG_SIZE);
    fmt::print("  Received {} bytes, x = {}, ID = {}\n", bytes_received, msg.x, (int)buf[0]);
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