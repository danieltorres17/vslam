#include <gtest/gtest.h>
#include "vslam/utils.h"

TEST(VslamUtils, UvInImage) {
  const int width = 512;
  const int height = 270;

  EXPECT_TRUE(vo::utils::UvInImage(width, height, gtsam::Point2(0, 0)));
  EXPECT_TRUE(vo::utils::UvInImage(width, height, gtsam::Point2(100, 100)));
  EXPECT_FALSE(vo::utils::UvInImage(width, height, gtsam::Point2(-1, 0)));
  EXPECT_FALSE(vo::utils::UvInImage(width, height, gtsam::Point2(width, height)));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
