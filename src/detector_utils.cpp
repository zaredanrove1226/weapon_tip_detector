#include "weapon_tip_detector/detector_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace weapon_tip_detector
{

double clamp01(const double v)
{
  return std::clamp(v, 0.0, 1.0);
}

bool validIntrinsics(const sensor_msgs::msg::CameraInfo & info)
{
  return info.k[0] > 1e-6 && info.k[4] > 1e-6;
}

double percentile(std::vector<float> values, const double p)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  std::sort(values.begin(), values.end());

  const double pp = std::clamp(p, 0.0, 1.0);
  const size_t idx = static_cast<size_t>(
    std::lround(pp * static_cast<double>(values.size() - 1)));

  return static_cast<double>(values.at(idx));
}

double closenessScore(
  const double value,
  const double ideal,
  const double tolerance)
{
  const double tol = std::max(1e-6, tolerance);
  return clamp01(1.0 - std::abs(value - ideal) / tol);
}

cv::Mat orMaskSafe(const cv::Mat & a, const cv::Mat & b)
{
  if (a.empty()) {
    return b.clone();
  }

  if (b.empty()) {
    return a.clone();
  }

  cv::Mat out;
  cv::bitwise_or(a, b, out);
  return out;
}

}  // namespace weapon_tip_detector