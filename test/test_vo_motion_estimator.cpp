#include <gtest/gtest.h>
#include "vo_test_utils.h"
#include "data_simulator.h"

struct VoFixture : public testing::Test {
protected:
  // Grid point parameters.
  const int num_rows = 40;
  const int num_cols = 16;
  const double grid_spacing = 0.4;

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

  const int num_traj_poses = 10;  // Number of trajectory poses to sample.

  // DataSimulator::Options object.
  DataSimulator::Options opts;

  void SetUp() override {
    // Populate options for data simulator.
    opts = {.img_width = width,
            .img_height = height,
            .filter_uvs_outside_img = true,
            .pose_world_map = pose_world_map,
            .pose_map_caml0 = pose_map_caml0,
            .caml_odom_delta_gt = pose_caml_delta_gt,
            .pose_caml_camr = pose_caml_camr,
            .xyz_std_dev = 0.02,
            .rpy_std_dev = 0.1,
            .num_frames = num_traj_poses,
            .should_sample_grid = true,
            .grid_sampling_prob = 0.5,
            .num_rows_grid = num_rows,
            .num_cols_grid = num_cols,
            .grid_spacing = grid_spacing};
  }
};

// Tests that the 3D visualization tool works.
TEST_F(VoFixture, CreateEnvironment) {
  // Create data simulator.
  DataSimulator sim(opts);

  // Get the left and right pixel measurements at the first pose.
  const auto measurements = sim.generateSetOfMeasurements(0, K);
  ASSERT_GT(measurements.size(), 0);
  const DataSimulator::MeasVectors meas_vectors = DataSimulator::ConvertMeasVecToVectors(measurements);

  // Plot and save projection images.
  const cv::Mat image_caml0 = PlotProjections(meas_vectors.uvs_left, width, height);
  const cv::Mat image_camr0 = PlotProjections(meas_vectors.uvs_right, width, height);
  SaveImage(image_caml0, "image_caml0.png");
  SaveImage(image_camr0, "image_camr0.png");

  // Create and visualize environment. (Disabled because the PCL visualization crashes out. Need to debug.)
  // VisualizeSetup(pose_world_map, sampled_pts3d_map_gt, frames_map_gt);
}

// Tests the stereo visual odometry front end. The odometry is solved in the map frame.
TEST_F(VoFixture, StereoVO) {
  // To store estimated poses.
  std::vector<gtsam::Pose3> odom_vec;
  odom_vec.reserve(opts.num_frames);

  // Create data simulator.
  DataSimulator sim(opts);
  const std::vector<VoFrame>& frames_map_gt = sim.framesMapGroundTruth();
  ASSERT_EQ(frames_map_gt.size(), opts.num_frames);

  // Loop through poses to estimate.
  for (size_t ii = 1; ii < opts.num_frames; ii++) {
    // Current left camera pose in map frame.
    const gtsam::Pose3& pose_map_caml_curr = frames_map_gt.at(ii).pose_map_caml;

    // Get measurements at second pose.
    const std::vector<vo::Measurement> measurements = sim.newUvsRigAtIdx(pose_map_caml_curr, K, ii);
    ASSERT_GT(measurements.size(), 0);
    const DataSimulator::MeasVectors meas_vectors = DataSimulator::ConvertMeasVecToVectors(measurements);

    ASSERT_FALSE(measurements.empty());

    // Visualize measurements.
    std::vector<gtsam::Point2> uvs_caml, uvs_camr;
    uvs_caml.reserve(measurements.size());
    uvs_camr.reserve(measurements.size());
    for (const auto& m : measurements) {
      uvs_caml.push_back(m.uv_left);
      uvs_camr.push_back(m.uv_right);
    }
    const cv::Mat image_caml0 = PlotProjections(meas_vectors.uvs_left, width, height);
    const cv::Mat image_camr0 = PlotProjections(meas_vectors.uvs_right, width, height);

    // Show images side by side.
    cv::Mat image_combined;
    cv::hconcat(image_caml0, image_camr0, image_combined);
    cv::imshow("Left and Right Camera Measurements", image_combined);
    cv::waitKey(0);

    // Generate noisy initial guess.
    gtsam::Pose3 odom_est = sim.noisyPoseEstimate(gtsam::Pose3());
    std::cout << "Initial guess: " << PoseVectorFmt(PoseToVector(odom_est)) << std::endl;

    // Covariance matrix - initially set to identity.
    gtsam::Matrix66 odom_est_cov = gtsam::I_6x6;

    // 3D points at previous left and right camera frames.
    const auto& [pts3d_caml_prev, pts3d_camr_prev] = sim.pts3dRigAtIdx(ii - 1);

    // Estimate odometry.
    vo::MotionEstimator estimator(width, height, true);
    const bool success = estimator.solve(measurements, pts3d_caml_prev, pts3d_camr_prev, K, pose_caml_camr,
                                         odom_est, odom_est_cov);
    EXPECT_TRUE(success);
    std::cout << "Optimized guess: " << PoseVectorFmt(PoseToVector(odom_est)) << "\n\n";
    EXPECT_FALSE(odom_est_cov.hasNaN());

    // Store estimated odometry.
    odom_vec.push_back(odom_est);

    // Compare estimated odometry to ground truth odometry.
    for (const auto& odom_est : odom_vec) {
      const gtsam::Pose3 odom_est_gt_delta = odom_est.between(pose_caml_delta_gt);
      const gtsam::Vector3& delta_xyz = odom_est_gt_delta.translation();
      const gtsam::Vector3 delta_rpy = odom_est_gt_delta.rotation().rpy();

      EXPECT_LT(delta_xyz.norm(), 1e-3);
      EXPECT_LT(delta_rpy.norm(), 1e-3);
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
