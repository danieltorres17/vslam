#pragma once

#include <gtsam/geometry/Pose3.h>
#include <random>

namespace vo {

class StereoTrajGenerator {
public:
  StereoTrajGenerator(const gtsam::Pose3& pose_init);
  virtual ~StereoTrajGenerator() {}
  virtual gtsam::Pose3 generatePose() = 0;
  gtsam::Pose3 poseInit() const { return pose_init_; }

private:
  const gtsam::Pose3 pose_init_;
};

/**
 * @brief Derived class of StereoTrajGenerator which generates a fixed pose delta every time generatePose() is
 * called. The current pose of the camera is stored and updated every time generatePose() is called. The
 * current pose of the camera is initialized to the given initial pose.
 *
 */
class FixedStereoTrajGenerator : public StereoTrajGenerator {
public:
  /**
   * @brief Construct a new Fixed Stereo Traj Generator object. The pose_curr_ is initialized to pose_init.
   * Every time generatePose() is called, pose_curr_ is updated by composing it with the fixed pose_delta_.
   *
   * @param pose_init initial pose of camera.
   * @param pose_delta fixed pose delta to apply to camera pose each time generatePose() is called.
   */
  FixedStereoTrajGenerator(const gtsam::Pose3& pose_init, const gtsam::Pose3& pose_delta);

  /**
   * @brief Return gtsam::Pose3 which represents the current pose of the camera composed with the fixed pose
   * delta. The current pose variable is updated every time this function is called.
   *
   * @return gtsam::Pose3
   */
  gtsam::Pose3 generatePose() override;
  gtsam::Pose3 poseDelta() const { return pose_delta_; }
  gtsam::Pose3 poseCurrent() const { return pose_curr_; }

private:
  const gtsam::Pose3 pose_delta_;
  gtsam::Pose3 pose_curr_;
};

/**
 * @brief Derived class of the StereoTrajGenerator which generates a random pose delta every time
 * generatePose() is called. The current pose of the camera is not stored here and must be stored externally.
 * The generated pose delta returned by generatePose() is a random pose delta with a random z component
 * between 0 and z_max. The rotation component of the pose delta is zero.
 *
 */
class RandomFwdStereoTrajGenerator : public StereoTrajGenerator {
public:
  RandomFwdStereoTrajGenerator(const gtsam::Pose3& pose_init, const double z_max = 0.5);
  /**
   * @brief Return gtsam::Pose3 which represents pose delta with a random z component between 0 and z_max. The
   * rotation component of the pose delta is zero. There is no change to the current pose variable.
   *
   * @return gtsam::Pose3
   */
  gtsam::Pose3 generatePose() override;

private:
  const double z_max_;
  std::default_random_engine generator_;
  std::uniform_real_distribution<double> z_distr_;
};

}  // namespace vo