#!/bin/bash

# --libraries src/slap src/crc8 src/vicon_packet
# --build-path bin
# --port string
# --warnings all

# Build the Test Sketch
# arduino-cli compile  --fqbn adafruit:samd:adafruit_feather_m0  --warnings all  --build-path bin  src/test

# Build the IMU Sketch
board_name=adafruit:samd:adafruit_feather_m0
# library=src/Adafruit_BNO055,src/Adafruit_Unified_Sensor,src/arduino-LoRa-Master,src/slap,src/crc8,src/vicon_packet
# library=src/Adafruit_BNO055,src/Adafruit_Unified_Sensor,src/arduino-LoRa-Master,src/slap,src/crc8,src/vicon_packet
# libraries=src/crc8
# library=src/crc8
library=src
libraries=src
build_target=src/test

arduino-cli compile  --fqbn $board_name  --warnings all  --build-path bin   --library $library  --libraries $libraries  $build_target

#  --library $library
#   --libraries $libraries