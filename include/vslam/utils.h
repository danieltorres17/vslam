#pragma once

#include <gtsam/geometry/Point2.h>

namespace vo {

namespace utils {

static bool UvInImage(const int width, const int height, const gtsam::Point2& uv) {
  return (uv.x() >= 0 && uv.x() < width && uv.y() >= 0 && uv.y() < height);
}

}  // namespace utils

}  // namespace vo
