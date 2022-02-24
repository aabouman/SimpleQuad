#!/bin/bash

# --libraries src/slap src/crc8 src/vicon_packet
# --build-path bin
# --port string
# --warnings all

# Build the Test Sketch
# arduino-cli compile  --fqbn adafruit:samd:adafruit_feather_m0  --warnings all  --build-path bin  src/test

# Build the IMU Sketch
# library=src/Adafruit_BNO055,src/Adafruit_Unified_Sensor,src/arduino-LoRa-Master,src/slap,src/crc8,src/vicon_packet
# library=src/Adafruit_BNO055,src/Adafruit_Unified_Sensor,src/arduino-LoRa-Master,src/slap,src/crc8,src/vicon_packet
# libraries=src/crc8
# library=src/crc8

build_path=bin
board_name=adafruit:samd:adafruit_feather_m0
libraries=src/utility
build_target=src/imu_vicon_feather
# compile_command="arduino-cli compile  --fqbn $board_name  --warnings all  --build-path bin  --libraries $libraries  $build_target"

# echo $compile_command
# $compile_command
arduino-cli compile  --fqbn $board_name  --warnings all  --build-path bin  --libraries $libraries  $build_target

# port=/dev/tty.usbmodem14101

# arduino-cli upload  --port $port  --input-dir