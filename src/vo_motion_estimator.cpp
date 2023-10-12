#include "vslam/vo_motion_estimator.h"
#include <gtsam/geometry/PinholeCamera.h>

namespace vo {

ReprojectionError::ReprojectionError(const int width, const int height, const gtsam::Point2& uv_left,
                                     const gtsam::Point2& uv_right, const gtsam::Cal3_S2& K,
                                     const gtsam::Point3& pt3d_caml, const gtsam::Point3& pt3d_camr,
                                     const gtsam::Pose3& pose_caml_camr)
  : width_(width)
  , height_(height)
  , uv_left_(uv_left)
  , uv_right_(uv_right)
  , K_(K)
  , pt3d_caml_(pt3d_caml)
  , pt3d_camr_(pt3d_camr)
  , pose_caml_camr_(pose_caml_camr) {}

bool ReprojectionError::operator()(const double* const pose_a_b_arr, double* residuals) const {
  // Convert left camera pose estimate into gtsam::Pose3.
  const gtsam::Pose3 pose_a_b_caml =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(pose_a_b_arr[3], pose_a_b_arr[4], pose_a_b_arr[5]),
                   gtsam::Point3(pose_a_b_arr[0], pose_a_b_arr[1], pose_a_b_arr[2]));

  // Project point into left camera.
  const auto& [uv_caml, uvl_safe] =
      gtsam::PinholeCamera<gtsam::Cal3_S2>(pose_a_b_caml, K_).projectSafe(pt3d_caml_);

  // Project point into right camera.
  const gtsam::Pose3 pose_a_b_camr = pose_a_b_caml.compose(pose_caml_camr_);
  const auto& [uv_camr, uvr_safe] =
      gtsam::PinholeCamera<gtsam::Cal3_S2>(pose_a_b_camr, K_).projectSafe(pt3d_camr_);

  if (!uvl_safe || !uvr_safe) {
    std::cout << "uv_caml: " << uv_caml.x() << ", " << uv_caml.y() << "\n";
    std::cout << "uv_camr: " << uv_camr.x() << ", " << uv_camr.y() << "\n";
    return false;
  }

  // Reprojection error.
  residuals[0] = uv_caml.x() - uv_left_.x();
  residuals[1] = uv_caml.y() - uv_left_.y();
  residuals[2] = uv_camr.x() - uv_right_.x();
  residuals[3] = uv_camr.y() - uv_right_.y();

  return true;
}

ceres::CostFunction* ReprojectionError::Create(const int width, const int height,
                                               const gtsam::Point2& uv_left, const gtsam::Point2& uv_right,
                                               const gtsam::Point3& pt3d_caml, const gtsam::Point3& pt3d_camr,
                                               const gtsam::Cal3_S2& K, const gtsam::Pose3& pose_caml_camr) {
  return new ceres::NumericDiffCostFunction<ReprojectionError, ceres::CENTRAL, 4, 6>(
      new ReprojectionError(width, height, uv_left, uv_right, K, pt3d_caml, pt3d_camr, pose_caml_camr));
}

bool ReprojectionError::UvInImage(const int width, const int height, const gtsam::Point2& uv) {
  return (uv.x() >= 0 && uv.x() < width && uv.y() >= 0 && uv.y() < height);
}

MotionEstimator::MotionEstimator(const int width, const int height, const bool verbose)
  : width_(width), height_(height) {
  // Set solver options.
  options_.max_num_iterations = 50;
  options_.linear_solver_type = ceres::DENSE_QR;
  options_.minimizer_progress_to_stdout = verbose ? true : false;
  options_.num_threads = 4;
  options_.parameter_tolerance = 1e-12;
}

MotionEstimator::~MotionEstimator() {}

bool MotionEstimator::solve(const std::vector<Measurement>& measurements,
                            const std::vector<gtsam::Point3>& pts3d_caml,
                            const std::vector<gtsam::Point3>& pts3d_camr, const gtsam::Cal3_S2& K,
                            const gtsam::Pose3& pose_caml_camr, gtsam::Pose3& pose_a_b,
                            gtsam::Matrix66& cov_mat) const {
  // Check the number of given 3D points.
  assert(pts3d_caml.size() == pts3d_camr.size() && "Number of 3D points must be equal.");

  // Create initial pose estimate array.
  const gtsam::Point3& xyz = pose_a_b.translation();
  const gtsam::Point3 rpy = pose_a_b.rotation().rpy();
  double pose_a_b_arr[6] = {xyz.x(), xyz.y(), xyz.z(), rpy.x(), rpy.y(), rpy.z()};

  // Create residuals and solve problem.
  ceres::Problem problem;
  for (const auto& meas : measurements) {
    auto cost_functor =
        ReprojectionError::Create(width_, height_, meas.uv_left, meas.uv_right, pts3d_caml.at(meas.pt3d_idx),
                                  pts3d_camr.at(meas.pt3d_idx), K, pose_caml_camr);
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(loss_scaling_param_);
    problem.AddResidualBlock(cost_functor, loss_function, pose_a_b_arr);
  }

  // Solve problem and update pose argument.
  ceres::Solver::Summary summary;
  ceres::Solve(options_, &problem, &summary);
  gtsam::Rot3 R = gtsam::Rot3::RzRyRx(pose_a_b_arr[3], pose_a_b_arr[4], pose_a_b_arr[5]);
  gtsam::Point3 t = {pose_a_b_arr[0], pose_a_b_arr[1], pose_a_b_arr[2]};
  pose_a_b = gtsam::Pose3(R, t);

  // Check if the problem successfully converged.
  if (summary.termination_type == ceres::FAILURE) {
    return false;
  }

  // Compute the covariance matrix.
  ceres::Covariance::Options cov_options;
  ceres::Covariance cov(cov_options);
  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  covariance_blocks.push_back(std::make_pair(pose_a_b_arr, pose_a_b_arr));

  if (!cov.Compute(covariance_blocks, &problem)) {
    cov_mat.setConstant(std::numeric_limits<double>::quiet_NaN());
    return false;
  }

  // Copy covariance matrix into output argument.
  cov.GetCovarianceBlock(pose_a_b_arr, pose_a_b_arr, cov_mat.data());

  return true;
}

}  // namespace vo
