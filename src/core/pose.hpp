#pragma once

#include <inttypes.h>

namespace rexlab {

struct PoseMsg {
    static constexpr uint8_t MsgID() { return 11; }
    float x;
    float y;
    float z;
    float qw;
    float qx;
    float qy;
    float qz;
};

void PoseToBytes(char* buf, const PoseMsg& pose);
void PoseFromBytes(PoseMsg& pose, const char* buf);

}  // namespace rexlab