#include "weapon_tip_detector/depth_projector.hpp"

#include "sensor_msgs/image_encodings.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace weapon_tip_detector
{

DepthProjector::DepthProjector(const DepthProjectorConfig & config)
: config_(config)
{
}

void DepthProjector::setConfig(const DepthProjectorConfig & config)
{
  config_ = config;
}

const DepthProjectorConfig & DepthProjector::config() const
{
  return config_;
}

bool DepthProjector::readDepthMeters(
  const cv::Mat & depth_image,
  const std::string & encoding,
  const int row,
  const int col,
  double & z_m) const
{
  if (encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    const uint16_t d = depth_image.at<uint16_t>(row, col);
    if (d == 0) {
      return false;
    }

    z_m = static_cast<double>(d) * 0.001;
    return std::isfinite(z_m) && z_m > 0.0;
  }

  if (encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    const float d = depth_image.at<float>(row, col);
    if (!std::isfinite(d) || d <= 0.0f) {
      return false;
    }

    z_m = static_cast<double>(d);
    return true;
  }

  return false;
}

void DepthProjector::depthPixelToDepthCamera(
  const int u,
  const int v,
  const double z,
  const sensor_msgs::msg::CameraInfo & depth_info,
  double & x_d,
  double & y_d,
  double & z_d)
{
  const double fx = depth_info.k[0];
  const double fy = depth_info.k[4];
  const double cx = depth_info.k[2];
  const double cy = depth_info.k[5];

  x_d = (static_cast<double>(u) - cx) * z / fx;
  y_d = (static_cast<double>(v) - cy) * z / fy;
  z_d = z;
}

void DepthProjector::depthCameraToColorCamera(
  const double x_d,
  const double y_d,
  const double z_d,
  const realsense2_camera_msgs::msg::Extrinsics & extrinsics,
  double & x_c,
  double & y_c,
  double & z_c)
{
  const auto & r = extrinsics.rotation;
  const auto & t = extrinsics.translation;

  // RealSense extrinsics rotation is column-major 3x3.
  x_c = r[0] * x_d + r[3] * y_d + r[6] * z_d + t[0];
  y_c = r[1] * x_d + r[4] * y_d + r[7] * z_d + t[1];
  z_c = r[2] * x_d + r[5] * y_d + r[8] * z_d + t[2];
}

bool DepthProjector::colorCameraToPixel(
  const double x_c,
  const double y_c,
  const double z_c,
  const sensor_msgs::msg::CameraInfo & color_info,
  int & u_c,
  int & v_c)
{
  if (z_c <= 1e-6) {
    return false;
  }

  const double fx = color_info.k[0];
  const double fy = color_info.k[4];
  const double cx = color_info.k[2];
  const double cy = color_info.k[5];

  u_c = static_cast<int>(std::lround(fx * x_c / z_c + cx));
  v_c = static_cast<int>(std::lround(fy * y_c / z_c + cy));

  return true;
}

bool DepthProjector::projectDepthToColor(
  const cv::Mat & depth_image,
  const std::string & encoding,
  const sensor_msgs::msg::CameraInfo & depth_info,
  const sensor_msgs::msg::CameraInfo & color_info,
  const realsense2_camera_msgs::msg::Extrinsics & extrinsics,
  const int color_width,
  const int color_height,
  cv::Mat & color_depth,
  cv::Mat & distance_mask) const
{
  if (encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
      encoding != sensor_msgs::image_encodings::TYPE_32FC1)
  {
    color_depth = cv::Mat(
      color_height,
      color_width,
      CV_32FC1,
      cv::Scalar(std::numeric_limits<float>::quiet_NaN()));

    distance_mask = cv::Mat::zeros(color_height, color_width, CV_8UC1);
    return false;
  }

  color_depth = cv::Mat(
    color_height,
    color_width,
    CV_32FC1,
    cv::Scalar(std::numeric_limits<float>::quiet_NaN()));

  distance_mask = cv::Mat::zeros(color_height, color_width, CV_8UC1);

  const double lower = std::max(
    config_.valid_depth_min,
    config_.target_distance - config_.distance_tolerance);

  const double upper = std::min(
    config_.valid_depth_max,
    config_.target_distance + config_.distance_tolerance);

  for (int v = 0; v < depth_image.rows; ++v) {
    for (int u = 0; u < depth_image.cols; ++u) {
      double z_m = 0.0;
      if (!readDepthMeters(depth_image, encoding, v, u, z_m)) {
        continue;
      }

      if (z_m < lower || z_m > upper) {
        continue;
      }

      double x_d = 0.0;
      double y_d = 0.0;
      double z_d = 0.0;

      depthPixelToDepthCamera(u, v, z_m, depth_info, x_d, y_d, z_d);

      double x_c = 0.0;
      double y_c = 0.0;
      double z_c = 0.0;

      depthCameraToColorCamera(x_d, y_d, z_d, extrinsics, x_c, y_c, z_c);

      int u_c = 0;
      int v_c = 0;

      if (!colorCameraToPixel(x_c, y_c, z_c, color_info, u_c, v_c)) {
        continue;
      }

      if (u_c < 0 || u_c >= color_width || v_c < 0 || v_c >= color_height) {
        continue;
      }

      float & old_z = color_depth.at<float>(v_c, u_c);

      if (!std::isfinite(old_z) || z_m < static_cast<double>(old_z)) {
        old_z = static_cast<float>(z_m);
      }

      distance_mask.at<uint8_t>(v_c, u_c) = 255;
    }
  }

  if (cv::countNonZero(distance_mask) > 0 && config_.mask_dilate_size > 1) {
    const int k = config_.mask_dilate_size;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    cv::dilate(distance_mask, distance_mask, kernel);
  }

  return true;
}

}  // namespace weapon_tip_detector