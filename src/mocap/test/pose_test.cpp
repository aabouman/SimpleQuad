#include <gtest/gtest.h>

#include "pose.hpp"


namespace rexlab {

TEST(PoseTests, Serialization) {
    PoseMsg pose = {1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};
    char buf[sizeof(PoseMsg)];
    PoseToBytes(buf, pose);
    PoseMsg pose2;
    PoseFromBytes(pose2, buf);
    EXPECT_FLOAT_EQ(pose.x, pose2.x);
    EXPECT_FLOAT_EQ(pose.y, pose2.y);
    EXPECT_FLOAT_EQ(pose.z, pose2.z);
    EXPECT_FLOAT_EQ(pose.qw, pose2.qw);
    EXPECT_FLOAT_EQ(pose.qx, pose2.qx);
    EXPECT_FLOAT_EQ(pose.qy, pose2.qy);
    EXPECT_FLOAT_EQ(pose.qz, pose2.qz);
}

}