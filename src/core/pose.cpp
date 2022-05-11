#include "pose.hpp"

#include "string.h"

namespace rexlab {

void PoseToBytes(char* buf, const PoseMsg& pose) {
    memcpy(buf, &pose, sizeof(PoseMsg));
}

void PoseFromBytes(PoseMsg& pose, const char* buf) {
    memcpy(&pose, buf, sizeof(PoseMsg));
}

}  // namespace rexlab