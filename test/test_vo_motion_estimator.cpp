#include <gtest/gtest.h>
#include "vo_test_utils.h"
#include "vslam/vo_motion_estimator.h"

#include <gtsam/geometry/PinholeCamera.h>

struct VoFixture : public testing::Test {
protected:
  // Grid point parameters.
  const int num_rows = 40;
  const int num_cols = 16;
  const double grid_spacing = 0.4;

  // Environment/sampled grid points in world frame.
  const std::vector<gtsam::Point3> pts3d_world_gt = CreateEnvironmentPts(num_rows, num_cols, grid_spacing);
  const std::vector<gtsam::Point3> sampled_pts3d_world_gt = SampleGridPts(pts3d_world_gt, 0.5);

  // Sampled points in map frame.
  std::vector<gtsam::Point3> sampled_pts3d_map_gt;

  // Camera extrinsics.
  const gtsam::Pose3 pose_caml_camr =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0., 0., 0.),
                   gtsam::Point3(0.5, 0.0, 0.0));  // Right camera pose in left camera frame.
  // Camera intrinsics.
  const int width = 512;
  const int height = 270;
  const gtsam::Cal3_S2 K = gtsam::Cal3_S2(200, 200, 0, width / 2.0, height / 2.0);

  // Map pose (left camera origin when solving for pose) in world frame.
  const gtsam::Pose3 pose_world_map = gtsam::Pose3(
      gtsam::Rot3::RzRyRx(-M_PI_2, 0., -M_PI_2),
      gtsam::Point3(-0.25, 0.5 * grid_spacing * (num_cols - 1), 0.5 * grid_spacing * (num_cols - 1)));

  // First left pose in map frame.
  const gtsam::Pose3 pose_map_caml0 = gtsam::Pose3();

  // Ground truth pose delta to be applied to the left camera between frames.
  const gtsam::Pose3 pose_caml_delta_gt =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0., 0., 0.), gtsam::Point3(0.0, 0.0, 1.5 * grid_spacing));

  const int num_traj_poses = 10;       // Number of trajectory poses to sample.
  std::vector<VoFrame> frames_map_gt;  // GT VO frames in the map frame.

  void SetUp() override {
    // Convert sampled ground truth 3D points into map frame.
    sampled_pts3d_map_gt.reserve(sampled_pts3d_world_gt.size());
    for (const auto& pt3d_w_gt : sampled_pts3d_world_gt) {
      sampled_pts3d_map_gt.push_back(pose_world_map.transformTo(pt3d_w_gt));
    }

    // Generate ground truth camera trajectory.
    frames_map_gt.reserve(num_traj_poses);
    const gtsam::Pose3 pose_map_camr0 = pose_map_caml0.compose(pose_caml_camr);
    frames_map_gt.push_back({.pose_map_caml = pose_map_caml0, .pose_map_camr = pose_map_camr0});
    for (int ii = 1; ii < num_traj_poses; ii++) {
      // Left camera pose in map frame.
      const gtsam::Pose3 pose_m_cl = frames_map_gt.at(ii - 1).pose_map_caml.compose(pose_caml_delta_gt);
      // Right camera pose in map frame.
      const gtsam::Pose3 pose_m_cr = pose_m_cl.compose(pose_caml_camr);

      // Add to ground truth trajectory.
      frames_map_gt.push_back({.pose_map_caml = pose_m_cl, .pose_map_camr = pose_m_cr});
    }
  }
};

// Tests that the 3D visualization tool works.
TEST_F(VoFixture, CreateEnvironment) {
  // Create cameras at second pose.
  const gtsam::Pose3& pose_m_cl = frames_map_gt.at(1).pose_map_caml;
  const gtsam::Pose3& pose_m_cr = frames_map_gt.at(1).pose_map_camr;
  const gtsam::PinholeCamera<gtsam::Cal3_S2> caml0(pose_m_cl, K);
  const gtsam::PinholeCamera<gtsam::Cal3_S2> camr0(pose_m_cr, K);

  // Project points into left and right cameras.
  std::vector<gtsam::Point2> uvs_caml;
  std::vector<gtsam::Point2> uvs_camr;
  for (const auto& pt3d_map : sampled_pts3d_map_gt) {
    // Safe project points into left and right cameras.
    const auto& [uv_caml, uvl_safe] = caml0.projectSafe(pt3d_map);
    const auto& [uv_camr, uvr_safe] = camr0.projectSafe(pt3d_map);
    if (!uvl_safe || !uvr_safe) {  // Move to next point if behind camera.
      continue;
    }
    uvs_caml.push_back(uv_caml);
    uvs_camr.push_back(uv_camr);
  }

  // Plot and save projection images.
  const cv::Mat image_caml0 = PlotProjections(uvs_caml, width, height);
  const cv::Mat image_camr0 = PlotProjections(uvs_camr, width, height);
  SaveImage(image_caml0, "image_caml0.png");
  SaveImage(image_camr0, "image_camr0.png");

  // Create and visualize environment.
  // VisualizeSetup(pose_world_map, sampled_pts3d_map_gt, frames_map_gt);
}

// Tests the stereo visual odometry front end. The odometry is solved in the map frame.
TEST_F(VoFixture, StereoVO) {
  // Number of poses to estimate poses for.
  const size_t num_poses_to_est = num_traj_poses;

  // To store estimated poses.
  std::vector<gtsam::Pose3> odom_vec;
  odom_vec.reserve(num_poses_to_est);

  // Loop through poses to estimate.
  for (size_t ii = 1; ii < num_poses_to_est; ii++) {
    // Previous left camera pose in map frame.
    const gtsam::Pose3& pose_map_caml_prev = frames_map_gt.at(ii - 1).pose_map_caml;

    // Convert sampled ground truth 3D points into previous left and right camera frame.
    std::vector<gtsam::Point3> sampled_pts3d_caml_prev;
    sampled_pts3d_caml_prev.reserve(sampled_pts3d_map_gt.size());
    for (const auto& pt3d_map_gt : sampled_pts3d_map_gt) {
      sampled_pts3d_caml_prev.push_back(pose_map_caml_prev.transformTo(pt3d_map_gt));
    }
    std::vector<gtsam::Point3> sampled_pts3d_camr_prev;
    sampled_pts3d_camr_prev.reserve(sampled_pts3d_map_gt.size());
    for (const auto& pt3d_caml : sampled_pts3d_caml_prev) {
      sampled_pts3d_camr_prev.push_back(pose_caml_camr.transformTo(pt3d_caml));
    }

    // Current left camera pose in map frame.
    const gtsam::Pose3& pose_map_caml_curr = frames_map_gt.at(ii).pose_map_caml;

    // Create cameras with pose relative to previous left camera frame.
    const gtsam::Pose3 odom_gt = pose_map_caml_prev.between(pose_map_caml_curr);
    gtsam::PinholeCamera<gtsam::Cal3_S2> caml(odom_gt, K);
    gtsam::PinholeCamera<gtsam::Cal3_S2> camr(odom_gt.compose(pose_caml_camr), K);

    // Get measurements at second pose.
    std::vector<vo::Measurement> measurements;
    for (size_t jj = 0; jj < sampled_pts3d_caml_prev.size(); jj++) {
      const gtsam::Point3& pt3d_caml_prev = sampled_pts3d_caml_prev.at(jj);
      const gtsam::Point3 pt3d_camr_prev = pose_caml_camr.transformTo(pt3d_caml_prev);

      // Safe project points into left and right cameras since some points could be behind the second camera.
      const auto [uv_caml, uvl_safe] = caml.projectSafe(pt3d_caml_prev);
      const auto [uv_camr, uvr_safe] = camr.projectSafe(pt3d_camr_prev);

      if (!uvl_safe || !uvr_safe) {  // Move to next point if behind camera.
        continue;
      }

      measurements.push_back(vo::Measurement(uv_caml, uv_camr, jj));
    }
    ASSERT_FALSE(measurements.empty());

    // Visualize measurements.
    std::vector<gtsam::Point2> uvs_caml, uvs_camr;
    uvs_caml.reserve(measurements.size());
    uvs_camr.reserve(measurements.size());
    for (const auto& m : measurements) {
      uvs_caml.push_back(m.uv_left);
      uvs_camr.push_back(m.uv_right);
    }
    const cv::Mat image_caml0 = PlotProjections(uvs_caml, width, height);
    const cv::Mat image_camr0 = PlotProjections(uvs_camr, width, height);
    // Show images side by side.
    cv::Mat image_combined;
    cv::hconcat(image_caml0, image_camr0, image_combined);
    cv::imshow("Left and Right Camera Measurements", image_combined);
    cv::waitKey(0);

    // Generate noisy initial guess.
    const double xyz_std_dev = 0.02;
    const double rot_std_dev = 0.05;
    gtsam::Pose3 odom_est = ApplyNoiseToPose(gtsam::Pose3::Identity(), xyz_std_dev, rot_std_dev);
    std::cout << "Initial guess: " << PoseVectorFmt(PoseToVector(odom_est)) << std::endl;
    gtsam::Matrix66 odom_est_cov = gtsam::I_6x6;

    // Estimate odometry.
    vo::MotionEstimator estimator(width, height, true);
    const bool success = estimator.solve(measurements, sampled_pts3d_caml_prev, sampled_pts3d_camr_prev, K,
                                         pose_caml_camr, odom_est, odom_est_cov);
    EXPECT_TRUE(success);
    std::cout << "Optimized guess: " << PoseVectorFmt(PoseToVector(odom_est)) << "\n\n";

    // Store estimated odometry.
    odom_vec.push_back(odom_est);
  }

  // Compare estimated odometry to ground truth odometry.
  for (const auto& odom_est : odom_vec) {
    const gtsam::Pose3 odom_est_gt_delta = odom_est.between(pose_caml_delta_gt);
    const gtsam::Vector3& delta_xyz = odom_est_gt_delta.translation();
    const gtsam::Vector3 delta_rpy = odom_est_gt_delta.rotation().rpy();

    EXPECT_LT(delta_xyz.norm(), 1e-2);
    EXPECT_LT(delta_rpy.norm(), 1e-2);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
