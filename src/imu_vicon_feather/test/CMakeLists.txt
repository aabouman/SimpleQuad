cmake_minimum_required(VERSION 3.17)

project(imu_vicon_feather_test VERSION 0.1 LANGUAGES C CXX)
include(CMakePrintHelpers)

set(LIB_DIR ${CMAKE_SOURCE_DIR}/../../../deps)
# include_directories(imu_vicon_test PUBLIC
#   $<BUILD_INTERFACE:${LIB_DIR}>
# )

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
add_executable(imu_vicon_test
    imu_vicon_test.cpp
)
target_link_libraries(imu_vicon_test
    PRIVATE
        ArduinoEigen
)
