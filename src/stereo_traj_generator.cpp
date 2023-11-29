#include "vslam/stereo_traj_generator.h"

namespace vo {

StereoTrajGenerator::StereoTrajGenerator(const gtsam::Pose3& pose_init) : pose_init_(pose_init) {}

FixedStereoTrajGenerator::FixedStereoTrajGenerator(const gtsam::Pose3& pose_init,
                                                   const gtsam::Pose3& pose_delta)
  : StereoTrajGenerator(pose_init), pose_delta_(pose_delta), pose_curr_(pose_init) {}

gtsam::Pose3 FixedStereoTrajGenerator::generatePose() {
  pose_curr_ = pose_curr_.compose(pose_delta_);
  return pose_curr_;
}

RandomFwdStereoTrajGenerator::RandomFwdStereoTrajGenerator(const gtsam::Pose3& pose_init, const double z_max)
  : StereoTrajGenerator(pose_init), z_distr_(0.0, 0.5), z_max_(z_max) {}

gtsam::Pose3 RandomFwdStereoTrajGenerator::generatePose() {
  // Generate random pose delta.
  const double z = z_distr_(generator_);
  const gtsam::Pose3 pose_delta = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, z));

  return pose_delta;
}

}  // namespace vo