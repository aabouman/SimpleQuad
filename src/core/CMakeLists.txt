add_library(quadcore
    pose.hpp
    pose.cpp

    utils.hpp
    utils.cpp

    serial.hpp
    serial.cpp
)
target_link_libraries(quadcore
    PUBLIC
    libserialport

    PRIVATE
    fmt::fmt
)
target_include_directories(quadcore
    INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/.."
)