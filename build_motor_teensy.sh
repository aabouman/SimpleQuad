#!/bin/bash

helpmenu()
{
cat << EOF
usage: $0 [-c|--compile] [-u|--upload] [-m|--monitor] [-h|--help]

Compiles, and uploads arduino sketch to feather.

OPTIONS:
    PARAM            The param
    -h|--help        Show this message
    -c|--compile     Compile arduino sketch
    -m|--monitor     Opens serial monitor with Arduino
EOF
}

# Text Colors
GREEN='\033[00;32m'
RED='\033[00;31m'
YELLOW='\033[00;33m'
NOCOLOR='\033[0m'

REL_DIR=`dirname "$0"`
# Sketch files
BOARD_NAME=teensy:avr:teensy40
BULID_TARGET=${REL_DIR}/src/motor_teensy
LIBRARIES=${REL_DIR}/deps
BUILD_PATH=${REL_DIR}/bin/motor_teensy
BUILD_CACHE_PATH=${BUILD_PATH}/cache

# Find board port and name
BOARD_PORT_INFO=$(arduino-cli board list | grep "${BOARD_NAME}")
IFS=' ' read -r -a array <<< "$BOARD_PORT_INFO"
BOARD_PORT="${array[0]}"
BOARD_TYPE="${array[5]} ${array[6]} ${array[7]}"


# compile $BOARD_NAME $BUILD_CACHE_PATH $BUILD_PATH $LIBRARIES $BULID_TARGET
compile()
{
    echo -e "${GREEN}Compiling Arduino Sketch${NOCOLOR} ${BULID_TARGET}."
    arduino-cli compile --warnings all  --fqbn $1  --build-cache-path $2  --build-path $3  --libraries $4  --build-property "compiler.c.elf.flags=-O3"  $5
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
            compile $BOARD_NAME $BUILD_CACHE_PATH $BUILD_PATH $LIBRARIES $BULID_TARGET
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
