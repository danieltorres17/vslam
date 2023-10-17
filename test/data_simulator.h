#pragma once

#include "vo_test_utils.h"
#include "vslam/vo_motion_estimator.h"
#include "vslam/utils.h"
#include <gtsam/geometry/PinholeCamera.h>

class DataSimulator {
public:
  struct Options {
    // Image dimensions.
    int img_width, img_height;
    bool filter_uvs_outside_img = true;

    // Poses to define the world to map frame.
    gtsam::Pose3 pose_world_map;
    // Initial pose of the left camera in the map frame.
    gtsam::Pose3 pose_map_caml0;
    // Ground truth left camera odometry delta between frames.
    gtsam::Pose3 caml_odom_delta_gt;
    // Camera extrinsics - right camera in left camera frame.
    gtsam::Pose3 pose_caml_camr;
    // Std. devs. to apply to initial pose estimates.
    double xyz_std_dev = 0.02;  // [m].
    double rpy_std_dev = 0.1;   // [rad].

    // Number of frames to sample.
    int num_frames;
    // Should randomly sample 3D points from the environment.
    bool should_sample_grid = true;
    double grid_sampling_prob = 0.5;

    // Grid parameters.
    int num_rows_grid = 40;
    int num_cols_grid = 16;
    double grid_spacing = 0.4;  // [m].
  };

  struct MeasVectors {
    std::vector<gtsam::Point2> uvs_left;
    std::vector<gtsam::Point2> uvs_right;
    std::vector<size_t> pt3d_idxs;
  };

  /**
   * @brief Construct a new Data Simulator object. Constructs 3D point ground truth data in the world and map
   * frames. Also constructs ground truth camera trajectories in the map frame.
   *
   * @param options DataSimulator options.
   */
  explicit DataSimulator(const Options& options) : options_(options) {
    // Generate the ground truth 3D points and apply sampling if enabled.
    pts3d_world_gt_ =
        CreateEnvironmentPts(options_.num_rows_grid, options_.num_cols_grid, options_.grid_spacing);
    if (options_.should_sample_grid) {
      pts3d_world_gt_ = SampleGridPts(pts3d_world_gt_, options_.grid_sampling_prob);
    }

    // Convert sampled ground truth 3D points into map frame.
    pts3d_map_gt_.reserve(pts3d_world_gt_.size());
    for (const auto& pt3d_w_gt : pts3d_world_gt_) {
      pts3d_map_gt_.push_back(options_.pose_world_map.transformTo(pt3d_w_gt));
    }

    // Create ground truth camera trajectories.
    frames_map_gt_.reserve(options_.num_frames);
    const gtsam::Pose3 pose_map_camr0 = options_.pose_map_caml0.compose(options_.pose_caml_camr);
    frames_map_gt_.push_back({.pose_map_caml = options_.pose_map_caml0, .pose_map_camr = pose_map_camr0});
    for (size_t ii = 1; ii < options_.num_frames; ii++) {
      // Left camera pose in map frame at current time.
      const gtsam::Pose3 pose_m_cl =
          frames_map_gt_.at(ii - 1).pose_map_caml.compose(options_.caml_odom_delta_gt);
      // Right camera pose in map frame at current time.
      const gtsam::Pose3 pose_m_cr = pose_m_cl.compose(options_.pose_caml_camr);

      // Add to ground truth trajectory.
      frames_map_gt_.push_back({.pose_map_caml = pose_m_cl, .pose_map_camr = pose_m_cr});
    }
  }

  /**
   * @brief Return the ground truth 3D points in the world frame.
   *
   * @return const std::vector<gtsam::Point3>&
   */
  const std::vector<gtsam::Point3>& pts3dWorldGroundTruth() const { return pts3d_world_gt_; }

  /**
   * @brief Return the ground truth 3D points in the map frame.
   *
   * @return const std::vector<gtsam::Point3>&
   */
  const std::vector<gtsam::Point3>& pts3dMapGroundTruth() const { return pts3d_map_gt_; }

  /**
   * @brief Return the ground truth camera trajectories in the map frame.
   *
   * @return const std::vector<VoFrame>&
   */
  const std::vector<VoFrame>& framesMapGroundTruth() const { return frames_map_gt_; }

  /**
   * @brief Return MeasVectors struct which splits up the given measurements vector into separate vectors.
   *
   * @param measurements VO measurements vector.
   * @return MeasVectors
   */
  static MeasVectors ConvertMeasVecToVectors(const std::vector<vo::Measurement>& measurements) {
    MeasVectors meas_vecs;
    meas_vecs.uvs_left.reserve(measurements.size());
    meas_vecs.uvs_right.reserve(measurements.size());
    meas_vecs.pt3d_idxs.reserve(measurements.size());

    for (const auto& meas : measurements) {
      meas_vecs.uvs_left.push_back(meas.uv_left);
      meas_vecs.uvs_right.push_back(meas.uv_right);
      meas_vecs.pt3d_idxs.push_back(meas.pt3d_idx);
    }
    return meas_vecs;
  }

  /**
   * @brief Return a vector of measurements at the given frame index. If the given frame index is invalid, an
   * empty vector is returned.
   *
   * @param frame_idx frame index to get measurements at.
   * @param K camera calibration.
   * @return std::vector<vo::Measurement>
   */
  std::vector<vo::Measurement> generateSetOfMeasurements(const size_t frame_idx,
                                                         const gtsam::Cal3_S2& K) const {
    // Check that frame index is valid.
    if (frame_idx < 0 || frame_idx >= options_.num_frames) {
      return {};
    }

    // Get the current frame.
    const auto& curr_frame = frames_map_gt_.at(frame_idx);
    const gtsam::Pose3& pose_m_cl = curr_frame.pose_map_caml;
    const gtsam::Pose3& pose_m_cr = curr_frame.pose_map_camr;

    // Construct cameras at current pose.
    const gtsam::PinholeCamera<gtsam::Cal3_S2> caml0(pose_m_cl, K);
    const gtsam::PinholeCamera<gtsam::Cal3_S2> camr0(pose_m_cr, K);

    // Project 3D points into left and right cameras.
    std::vector<vo::Measurement> measurements;
    for (size_t ii = 0; ii < pts3d_map_gt_.size(); ii++) {
      const gtsam::Point3& pt3d_map = pts3d_map_gt_.at(ii);

      // Safe project points into left and right cameras.
      const auto& [uv_caml, uvl_safe] = caml0.projectSafe(pt3d_map);
      const auto& [uv_camr, uvr_safe] = camr0.projectSafe(pt3d_map);
      if (!uvl_safe || !uvr_safe) {  // Move to next point if behind camera.
        continue;
      }
      // Filter out projections if outside image.
      if (!vo::utils::UvInImage(options_.img_width, options_.img_height, uv_caml) ||
          !vo::utils::UvInImage(options_.img_width, options_.img_height, uv_camr)) {
        continue;
      }

      // Add measurement.
      measurements.emplace_back(uv_caml, uv_camr, ii);
    }

    return measurements;
  }

  /**
   * @brief Return a pair of vectors of 3D points in the left and right camera frames at the given frame
   * index.
   *
   * @param frame_idx frame index.
   * @return std::pair<std::vector<gtsam::Point3>, std::vector<gtsam::Point3>>
   */
  std::pair<std::vector<gtsam::Point3>, std::vector<gtsam::Point3>> pts3dRigAtIdx(
      const size_t frame_idx) const {
    // Check that frame index is valid.
    if (frame_idx < 0 || frame_idx >= options_.num_frames) {
      return {{}, {}};
    }

    // Left camera pose at frame index.
    const gtsam::Pose3& pose_m_cl = frames_map_gt_.at(frame_idx).pose_map_caml;

    // Convert sampled ground truth 3D points into indexed left and right camera frames.
    std::vector<gtsam::Point3> pts3d_caml;
    pts3d_caml.reserve(pts3d_map_gt_.size());
    for (const auto& pt3d_map_gt : pts3d_map_gt_) {
      pts3d_caml.push_back(pose_m_cl.transformTo(pt3d_map_gt));
    }

    std::vector<gtsam::Point3> pts3d_camr;
    pts3d_camr.reserve(pts3d_map_gt_.size());
    for (const auto& pt3d_caml : pts3d_caml) {
      pts3d_camr.push_back(options_.pose_caml_camr.transformTo(pt3d_caml));
    }

    return {pts3d_caml, pts3d_camr};
  }

  /**
   * @brief Return vector of vo::Measurement objects representing the pixel measurements taken at frame index
   * of the 3D points at frame index - 1. If the frame index or frame index - 1 is invalid, a pair with empty
   * vectors is returned.
   *
   * @param pose_map_caml_fidx pose of left camera at frame index in map frame.
   * @param K camera calibration.
   * @param frame_idx frame index. Both frame_idx and frame_idx - 1 must be valid.
   * @return std::vector<vo::Measurement>
   */
  std::vector<vo::Measurement> newUvsRigAtIdx(const gtsam::Pose3& pose_map_caml_fidx, const gtsam::Cal3_S2& K,
                                              const size_t frame_idx) const {
    // Check that frame indices is valid.
    size_t fidx = frame_idx;
    if ((fidx < 0) || (fidx - 1 < 0) || (fidx >= options_.num_frames)) {
      return {};
    }

    // 3D points in left and right camera at previous index.
    const auto& [pts3d_caml_prev, pts3d_camr_prev] = pts3dRigAtIdx(fidx - 1);

    // Create cameras using odometry delta.
    gtsam::PinholeCamera<gtsam::Cal3_S2> caml(options_.caml_odom_delta_gt, K);
    gtsam::PinholeCamera<gtsam::Cal3_S2> camr(options_.caml_odom_delta_gt.compose(options_.pose_caml_camr),
                                              K);

    // Project 3D points at previous index using indexed camera pose.
    std::vector<vo::Measurement> measurements;
    for (size_t ii = 0; ii < pts3d_caml_prev.size(); ii++) {
      const gtsam::Point3& pt3d_caml_prev = pts3d_caml_prev.at(ii);
      const gtsam::Point3 pt3d_camr_prev = options_.pose_caml_camr.transformTo(pt3d_caml_prev);

      // Safe project points into left and right cameras since some points could be behind the second camera.
      const auto [uv_caml, uvl_safe] = caml.projectSafe(pt3d_caml_prev);
      const auto [uv_camr, uvr_safe] = camr.projectSafe(pt3d_camr_prev);

      if (!uvl_safe || !uvr_safe) {  // Move to next point if behind camera.
        continue;
      }

      // Filter out projections if outside image.
      if (!vo::utils::UvInImage(options_.img_width, options_.img_height, uv_caml) ||
          !vo::utils::UvInImage(options_.img_width, options_.img_height, uv_camr)) {
        continue;
      }

      measurements.emplace_back(uv_caml, uv_camr, ii);
    }

    return measurements;
  }

  /**
   * @brief Return noisy pose estimate using the translation and rotation std. deviations defined in the
   * options.
   *
   * @param pose_init initial pose to perturb.
   * @return gtsam::Pose3
   */
  gtsam::Pose3 noisyPoseEstimate(const gtsam::Pose3& pose_init) const {
    return ApplyNoiseToPose(pose_init, options_.xyz_std_dev, options_.rpy_std_dev);
  }

private:
  const Options options_;
  std::vector<gtsam::Point3> pts3d_world_gt_;  // 3D points in world frame.
  std::vector<gtsam::Point3> pts3d_map_gt_;    // 3D points in map frame.
  std::vector<VoFrame> frames_map_gt_;         // Camera trajectory in map frame.
};
