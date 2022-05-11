add_library(quadcore
    pose.hpp
    pose.cpp

    utils.hpp
    utils.cpp
)
target_include_directories(quadcore
    INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/.."
)