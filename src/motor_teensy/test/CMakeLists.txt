cmake_minimum_required(VERSION 3.17)

project(motor_teensy_test VERSION 0.1 LANGUAGES C CXX)
include(CMakePrintHelpers)

set(LIB_DIR ${CMAKE_SOURCE_DIR}/../../../deps)

# Track down the ArduinoEigen library
set(ARDUINO_EIGEN_DIR ${LIB_DIR}/ArduinoEigen)
find_path(ARDUINO_EIGEN_INCLUDE_DIRS
    NAMES ArduinoEigen.h
    PATHS ${ARDUINO_EIGEN_DIR}
    NO_DEFAULT_PATH
    REQUIRED
)
set(ArduinoEigen_ROOT_DIR ${ARDUINO_EIGEN_INCLUDE_DIRS})
set(ArduinoEigen_INCLUDE_DIRS ${ARDUINO_EIGEN_INCLUDE_DIRS})
cmake_print_variables(ARDUINO_EIGEN_INCLUDE_DIRS)

# Build Arudino Eigen target
add_library(ArduinoEigen INTERFACE)
target_include_directories(ArduinoEigen
    INTERFACE
        ${ArduinoEigen_INCLUDE_DIRS}
)

# Build the executable
add_executable(motor_teensy_test
    motor_teensy_test.cpp
)
target_link_libraries(motor_teensy_test
    PRIVATE
        ArduinoEigen
)
