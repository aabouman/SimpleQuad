#include "pose.hpp"

#include "string.h"

namespace rexlab {

void PoseToBytes(char* buf, const PoseMsg& pose) {
    memcpy(buf+1, &pose, sizeof(PoseMsg));
    buf[0] = PoseMsg::MsgID();
}

bool PoseFromBytes(PoseMsg& pose, const char* buf) {
    memcpy(&pose, buf+1, sizeof(PoseMsg));
    return buf[0] == PoseMsg::MsgID();
}

}  // namespace rexlab