#!/bin/bash

helpmenu()
{
cat << EOF
usage: $0 PARAM [-o|--option OPTION] [-f|--force] [-h|--help]

Compiles, and uploads arduino sketch to feather.

OPTIONS:
   PARAM            The param
   -h|--help        Show this message
   -m|--monitor     Opens serial monitor with Arduino
EOF
}

# Text Colors
GREEN='\033[00;32m'
RED='\033[00;31m'
YELLOW='\033[00;33m'
NOCOLOR='\033[0m'

# Sketch files
BUILD_PATH=bin
BOARD_NAME=adafruit:samd:adafruit_feather_m0
LIBRARIES=src/utility
BULID_TARGET=src/imu_vicon_feather

# compile $BOARD_NAME $BUILD_PATH $LIBRARIES $BULID_TARGET
compile() # Board Name, Build Path, Libraries, Build Target
{
    arduino-cli compile --warnings all  --fqbn $BOARD_NAME  --build-path $BUILD_PATH  --libraries $LIBRARIES  $BULID_TARGET
}


# Compile Commands

# Upload Commands
BOARD_PORT_INFO=$(arduino-cli board list | grep "adafruit:samd:adafruit_feather_m0")
if [ "$BOARD_PORT_INFO" = "" ];
then
    echo -e "${RED}Feather not found, skipping upload."
else
    IFS=' ' read -r -a array <<< "$BOARD_PORT_INFO"
    BOARD_PORT="${array[0]}"
    BOARD_TYPE="${array[5]} ${array[6]} ${array[7]}"

    echo -e "Found ${GREEN}${BOARD_TYPE}${NOCOLOR} at port ${GREEN}${BOARD_PORT}${NOCOLOR}"
    echo -e "${YELLOW}Uploading..."
    arduino-cli upload --port ${BOARD_PORT} --fqbn ${BOARD_NAME} --input-dir ${BUILD_PATH} --verify ${BULID_TARGET}
    echo -e "${NOCOLOR}"


    while [ ! $# -eq 0 ]
    do
        case "$1" in
            --help | -h)
                helpmenu
                exit
                ;;
            --monitor | -m)
                echo -e "${GREEN}Opening monitor${NOCOLOR}"
                arduino-cli monitor --port ${BOARD_PORT}
                exit
                ;;
            *)
                helpmenu
                exit
                ;;
        esac
        shift
    done
fi
