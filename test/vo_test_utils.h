#pragma once

#include <iomanip>
#include <string>

#include <gtsam/geometry/Pose3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>

struct VoFrame {
  gtsam::Pose3 pose_map_caml;
  gtsam::Pose3 pose_map_camr;
};

static double DegToRad(const double angle_deg) { return angle_deg * M_PI / 180.0; }

static double RadToDeg(const double angle_rad) { return angle_rad * 180.0 / M_PI; }

std::vector<gtsam::Point3> CreateEnvironmentPts(const int num_rows = 40, const int num_cols = 16,
                                                const double grid_spacing = 0.4) {
  std::vector<gtsam::Point3> pts3d_world;

  // Set a bottom corner as origin.
  const gtsam::Pose3 origin = gtsam::Pose3();

  // Lambda to get grid of points.
  auto CreateGrid = [](const int num_rows, const int num_cols,
                       const double grid_spacing) -> std::vector<gtsam::Point3> {
    std::vector<gtsam::Point3> pts3d;
    for (int ii = 0; ii < num_rows; ii++) {
      for (int jj = 0; jj < num_cols; jj++) {
        pts3d.push_back(gtsam::Point3(ii * grid_spacing, jj * grid_spacing, 0));
      }
    }
    return pts3d;
  };

  // Enviroment consists of 3 grids: one floor and two walls.
  const std::vector<gtsam::Point3> floor_pts3d = CreateGrid(num_rows, num_cols, grid_spacing);
  std::vector<gtsam::Point3> wall1_pts3d = CreateGrid(num_rows, num_cols, grid_spacing);
  std::vector<gtsam::Point3> wall2_pts3d = CreateGrid(num_rows, num_cols, grid_spacing);

  // Transform wall1 points to left side.
  const gtsam::Rot3 R_floor_wall = gtsam::Rot3::RzRyRx(M_PI / 2, 0, 0);
  const gtsam::Point3 t_floor_leftwall = gtsam::Point3(0, grid_spacing * (num_cols - 1), grid_spacing);
  const gtsam::Pose3 T_floor_leftwall(R_floor_wall, t_floor_leftwall);
  std::for_each(wall1_pts3d.begin(), wall1_pts3d.end(),
                [&T_floor_leftwall](gtsam::Point3& pt3d) { pt3d = T_floor_leftwall.transformFrom(pt3d); });

  // Transform wall2 points to right side.
  const gtsam::Point3 t_floor_rightwall = gtsam::Point3(0, 0, grid_spacing);
  const gtsam::Pose3 T_floor_rightwall(R_floor_wall, t_floor_rightwall);
  std::for_each(wall2_pts3d.begin(), wall2_pts3d.end(),
                [&T_floor_rightwall](gtsam::Point3& pt3d) { pt3d = T_floor_rightwall.transformFrom(pt3d); });

  // Combine all points.
  pts3d_world.insert(pts3d_world.end(), floor_pts3d.begin(), floor_pts3d.end());
  pts3d_world.insert(pts3d_world.end(), wall1_pts3d.begin(), wall1_pts3d.end());
  pts3d_world.insert(pts3d_world.end(), wall2_pts3d.begin(), wall2_pts3d.end());

  return pts3d_world;
}

std::vector<gtsam::Point3> SampleGridPts(const std::vector<gtsam::Point3>& pts3d_world,
                                         const double pts3d_frac) {
  std::vector<gtsam::Point3> sampled_pts3d_world;

  // Uniformly sample points.
  static std::default_random_engine gen;
  std::uniform_int_distribution<int> distribution(0, pts3d_world.size() - 1);

  // Number of points to sample.
  const int num_pts3d_sampled = static_cast<int>(pts3d_frac * pts3d_world.size());

  // Sample points.
  for (int ii = 0; ii < num_pts3d_sampled; ii++) {
    const int idx = distribution(gen);
    sampled_pts3d_world.push_back(pts3d_world.at(idx));
  }

  return sampled_pts3d_world;
}

gtsam::Pose3 ApplyNoiseToPose(const gtsam::Pose3& pose, const double xyz_std_dev, const double rot_std_dev) {
  // Create random number generators.
  static std::default_random_engine gen;
  std::normal_distribution<double> xyz_dist(0.0, xyz_std_dev);
  std::normal_distribution<double> rot_dist(0.0, rot_std_dev);

  // Apply noise to translation.
  const gtsam::Vector3& xyz = pose.translation();
  const gtsam::Vector3 xyz_noisy = xyz + gtsam::Vector3(xyz_dist(gen), xyz_dist(gen), xyz_dist(gen));

  // Apply noise to rotation.
  const gtsam::Rot3 R = pose.rotation();
  const gtsam::Rot3 R_noisy =
      gtsam::Rot3::Expmap(gtsam::Vector3(rot_dist(gen), rot_dist(gen), rot_dist(gen))) * R;

  return gtsam::Pose3(R_noisy, xyz_noisy);
}

void AddPose3s(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<gtsam::Pose3>& poses,
               const double axis_scale = 1.0) {
  // Add camera poses to visualization.
  for (const auto& pose : poses) {
    // Convert pose to Eigen Matrix.
    const Eigen::Matrix4f pose_mat = pose.matrix().cast<float>();
    const Eigen::Affine3f pose_affine(pose_mat);
    viewer->addCoordinateSystem(axis_scale, pose_affine);
  }
}

void VisualizeSetup(const gtsam::Pose3& pose_world_map, const std::vector<gtsam::Point3>& pts3d_map,
                    const std::vector<VoFrame>& frames_map) {
  using namespace std::chrono_literals;

  // Viewer.
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);

  // Cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Helper to plot points.
  auto Add3dPts = [&viewer, &cloud](const std::vector<gtsam::Point3>& pts3d) -> void {
    // Add points to cloud.
    for (const auto& pt3d : pts3d) {
      pcl::PointXYZ pt_temp;
      pt_temp.x = pt3d.x();
      pt_temp.y = pt3d.y();
      pt_temp.z = pt3d.z();
      cloud->points.push_back(pt_temp);
    }
    viewer->addPointCloud(cloud, "PointCloud");
  };

  // Convert 3D points to world frame and add to visualization.
  std::vector<gtsam::Point3> pts3d_world;
  pts3d_world.reserve(pts3d_map.size());
  for (const auto& pt3d_m : pts3d_map) {
    pts3d_world.push_back(pose_world_map.transformFrom(pt3d_m));
  }
  Add3dPts(pts3d_world);

  // Add ground truth and estimated camera poses to visualization.
  AddPose3s(viewer, {gtsam::Pose3()}, 0.6);

  // Convert frames to world frame and add to visualization.
  for (const auto& frame_m : frames_map) {
    AddPose3s(viewer,
              {pose_world_map.compose(frame_m.pose_map_caml), pose_world_map.compose(frame_m.pose_map_camr)},
              0.6);
  }

  // Increase point size to 3.
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "PointCloud");

  while (!viewer->wasStopped()) {
    viewer->spin();
    std::this_thread::sleep_for(100ms);
  }
}

gtsam::Vector6 PoseToVector(const gtsam::Pose3& pose) {
  const gtsam::Vector3& xyz_vec = pose.translation();
  const gtsam::Vector3 rot_vec = pose.rotation().rpy();
  return (gtsam::Vector6() << xyz_vec.x(), xyz_vec.y(), xyz_vec.z(), rot_vec.x(), rot_vec.y(), rot_vec.z())
      .finished();
}

std::string PoseVectorFmt(const gtsam::Vector6& pose_vec) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(4) << pose_vec.transpose();
  return ss.str();
}

cv::Mat PlotProjections(const std::vector<gtsam::Point2>& uvs, const int width, const int height) {
  // Create an all-black image
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

  // Loop through the pixels and plot them as circles
  int radius = 2;                   // Adjust the radius as needed
  cv::Scalar color(255, 255, 255);  // White color

  for (const gtsam::Point2& pixel : uvs) {
    // Convert gtsam::Point2 to cv::Point
    cv::Point cvPoint(static_cast<int>(pixel.x()), static_cast<int>(pixel.y()));

    // Plot a circle at the pixel location
    cv::circle(image, cvPoint, radius, color, -1);  // -1 for a filled circle
  }

  return image;
}

bool SaveImage(const cv::Mat& image, const std::string& filename) {
  if (image.empty()) {
    std::cerr << "ERROR: Image is empty." << std::endl;
    return false;
  }

  return cv::imwrite(filename, image);
}