#pragma once

#include <ceres/ceres.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3_S2.h>

namespace vo {

struct Measurement {
  const gtsam::Point2 uv_left;
  const gtsam::Point2 uv_right;
  const size_t pt3d_idx;

  Measurement(const gtsam::Point2& uv_caml, const gtsam::Point2& uv_camr, const size_t pt3d_index)
    : uv_left(uv_caml), uv_right(uv_camr), pt3d_idx(pt3d_index) {}
};

class ReprojectionError {
public:
  /**
   * @brief Construct a new Reprojection Error object.
   *
   * @param uv_left measurement in left camera frame.
   * @param uv_right measurement in right camera frame.
   * @param K camera calibration.
   * @param pt3d_caml 3D point in left camera frame.
   * @param pt3d_camr 3D point in right camera frame.
   * @param pose_caml_camr pose of right camera frame relative to left camera frame (extrinsics).
   */
  ReprojectionError(const gtsam::Point2& uv_left, const gtsam::Point2& uv_right, const gtsam::Cal3_S2& K,
                    const gtsam::Point3& pt3d_caml, const gtsam::Point3& pt3d_camr,
                    const gtsam::Pose3& pose_caml_camr);

  bool operator()(const double* const pose_a_b_arr, double* residuals) const;

  static ceres::CostFunction* Create(const gtsam::Point2& uv_left, const gtsam::Point2& uv_right,
                                     const gtsam::Point3& pt3d_caml, const gtsam::Point3& pt3d_camr,
                                     const gtsam::Cal3_S2& K, const gtsam::Pose3& pose_caml_camr);

private:
  const gtsam::Point2 uv_left_;
  const gtsam::Point2 uv_right_;
  const gtsam::Cal3_S2 K_;
  const gtsam::Point3 pt3d_caml_;
  const gtsam::Point3 pt3d_camr_;
  const gtsam::Pose3 pose_caml_camr_;
};

class MotionEstimator {
public:
  /**
   * @brief Construct a new Motion Estimator object.
   *
   * @param width image width [px].
   * @param height image height [px].
   * @param verbose flag to set Ceres to verbose if true.
   */
  MotionEstimator(const int width, const int height, const bool verbose = false);
  ~MotionEstimator();

  /**
   * @brief Return true if odometry estimate was successfully calculated. Return false otherwise. The
   * residuals are constructed after filtering out the measured pixel coordinates outside the image
   * dimensions. The covariance matrix is updated if it was successfully calculated. Otherwise, if the
   * odometry estimate fails or the covariance matrix calculation fails, it will be set to contain all NaNs.
   * This method assumes that pts3d_caml and pts3d_camr are the same length.
   *
   * @param measurements vector of vo::Measurement objects.
   * @param pts3d_caml 3D points in left camera frame.
   * @param pts3d_camr 3D points in right camera frame.
   * @param K camera calibration matrix.
   * @param pose_caml_camr pose of right camera frame relative to left camera frame (extrinsics).
   * @param pose_a_b odometry estimate.
   * @param cov_mat covariance matrix of odometry estimate.
   * @return true
   * @return false
   */
  bool solve(const std::vector<Measurement>& measurements, const std::vector<gtsam::Point3>& pts3d_caml,
             const std::vector<gtsam::Point3>& pts3d_camr, const gtsam::Cal3_S2& K,
             const gtsam::Pose3& pose_caml_camr, gtsam::Pose3& pose_a_b, gtsam::Matrix66& cov_mat) const;

private:
  ceres::Solver::Options options_;
  const double loss_scaling_param_ = 1.0;
  const int width_;
  const int height_;
};

}  // namespace vo
