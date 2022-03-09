#!/bin/bash

helpmenu()
{
cat << EOF
usage: $0 [-f|--monitor] [-h|--help]

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
LIBRARIES=deps
BULID_TARGET=src/imu_vicon_feather

# Find board port and name
BOARD_PORT_INFO=$(arduino-cli board list | grep "${BOARD_NAME}")
IFS=' ' read -r -a array <<< "$BOARD_PORT_INFO"
BOARD_PORT="${array[0]}"
BOARD_TYPE="${array[5]} ${array[6]} ${array[7]}"


# compile $BOARD_NAME $BUILD_PATH $LIBRARIES $BULID_TARGET
compile()
{
    echo -e "${GREEN}Compiling Sketch.${NOCOLOR}"
    arduino-cli compile --warnings all  --fqbn $1  --build-path $2  --libraries $3  $4
}

# upload $BOARD_PORT $BOARD_NAME $BUILD_PATH $BULID_TARGET
upload()
{
    if [ "$1" = "" ];
    then
        echo -e "${RED}Feather not found, skipping upload.${NOCOLOR}"
    else
        arduino-cli upload --port $1 --fqbn $2 --input-dir $3  $4
    fi
}

# monitor $BOARD_PORT
monitor()
{
    if [ "$1" = "" ];
    then
        echo -e "${RED}Feather not found, skipping upload.${NOCOLOR}"
    else
        echo -e "${GREEN}Opening monitor${NOCOLOR}"
        arduino-cli monitor --port $1
    fi
}

# Param parser and function running
while [ ! $# -eq 0 ]
do
    case "$1" in
        --help | -h)
            helpmenu
            exit
            ;;
        --compile | -c)
            compile $BOARD_NAME $BUILD_PATH $LIBRARIES $BULID_TARGET
            ;;
        --upload | -u)
            upload $BOARD_PORT $BOARD_NAME $BUILD_PATH $BULID_TARGET
            ;;
        --monitor | -m)
            monitor $BOARD_PORT
            ;;
        *)
            helpmenu
            exit
            ;;
    esac
    shift
done
