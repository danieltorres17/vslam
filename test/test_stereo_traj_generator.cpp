#include <gtest/gtest.h>

#include "vslam/stereo_traj_generator.h"

struct StereoTrajGeneratorFixture : public testing::Test {
protected:
  const gtsam::Pose3 pose_init =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.1, 0.2, 0.3), gtsam::Point3(0.1, -0.2, 0.0));

  void SetUp() override {}
};

// Tests the FixedStereoTrajGenerator class.
TEST_F(StereoTrajGeneratorFixture, FixedStereoTrajGenerator) {
  const gtsam::Pose3 pose_delta =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.1, 0.2, 0.3), gtsam::Point3(0.1, -0.2, 0.0));
  vo::FixedStereoTrajGenerator traj_gen(pose_init, pose_delta);

  // Check that poseInit() returns the correct initial pose.
  EXPECT_TRUE(pose_init.equals(traj_gen.poseInit()));

  // Check that poseDelta() returns the correct pose delta.
  EXPECT_TRUE(pose_delta.equals(traj_gen.poseDelta()));

  // Check that poseCurrent() returns the correct current pose before calling generatePose() method.
  EXPECT_TRUE(pose_init.equals(traj_gen.poseCurrent()));

  // Check that generatePose() returns the correct pose.
  const gtsam::Pose3 pose_curr = traj_gen.generatePose();
  EXPECT_TRUE(pose_curr.equals(pose_init.compose(pose_delta)));
  EXPECT_TRUE(pose_curr.equals(traj_gen.poseCurrent()));
}

// Tests the RandomFwdStereoTrajGenerator class.
TEST_F(StereoTrajGeneratorFixture, RandomFwdStereoTrajGenerator) {
  const double z_max = 0.5;
  vo::RandomFwdStereoTrajGenerator traj_gen(pose_init, z_max);

  // Check that poseInit() returns the correct initial pose.
  EXPECT_TRUE(pose_init.equals(traj_gen.poseInit()));

  // Check that generatePose() returns the correct pose.
  const gtsam::Pose3 pose_curr = traj_gen.generatePose();
  const gtsam::Vector3 xyz_delta = pose_curr.translation() - pose_init.translation();
  EXPECT_GT(xyz_delta.z(), 0.0);
  EXPECT_TRUE(xyz_delta.z() <= z_max);
  EXPECT_GT(xyz_delta.norm(), 0.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}