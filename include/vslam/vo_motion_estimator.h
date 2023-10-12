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
  ReprojectionError(const int width, const int height, const gtsam::Point2& uv_left,
                    const gtsam::Point2& uv_right, const gtsam::Cal3_S2& K, const gtsam::Point3& pt3d_caml,
                    const gtsam::Point3& pt3d_camr, const gtsam::Pose3& pose_caml_camr);

  bool operator()(const double* const pose_a_b_arr, double* residuals) const;

  static ceres::CostFunction* Create(const int width, const int height, const gtsam::Point2& uv_left,
                                     const gtsam::Point2& uv_right, const gtsam::Point3& pt3d_caml,
                                     const gtsam::Point3& pt3d_camr, const gtsam::Cal3_S2& K,
                                     const gtsam::Pose3& pose_caml_camr);

  static bool UvInImage(const int width, const int height, const gtsam::Point2& uv);

private:
  const gtsam::Point2 uv_left_;
  const gtsam::Point2 uv_right_;
  const gtsam::Cal3_S2 K_;
  const gtsam::Point3 pt3d_caml_;
  const gtsam::Point3 pt3d_camr_;
  const gtsam::Pose3 pose_caml_camr_;
  const int width_;
  const int height_;
};

class MotionEstimator {
public:
  MotionEstimator(const int width, const int height, const bool verbose = false);
  ~MotionEstimator();

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
