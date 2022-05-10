#pragma once

struct PoseMsg {
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