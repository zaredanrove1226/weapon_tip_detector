#ifndef WEAPON_TIP_DETECTOR__DEPTH_PROJECTOR_HPP_
#define WEAPON_TIP_DETECTOR__DEPTH_PROJECTOR_HPP_

#include "sensor_msgs/msg/camera_info.hpp"
#include "realsense2_camera_msgs/msg/extrinsics.hpp"

#include <opencv2/core.hpp>

#include <string>

namespace weapon_tip_detector
{

struct DepthProjectorConfig
{
  double target_distance{0.30};
  double distance_tolerance{0.18};
  double valid_depth_min{0.12};
  double valid_depth_max{0.70};
  int mask_dilate_size{5};
};

class DepthProjector
{
public:
  explicit DepthProjector(const DepthProjectorConfig & config);

  void setConfig(const DepthProjectorConfig & config);

  const DepthProjectorConfig & config() const;

  bool projectDepthToColor(
    const cv::Mat & depth_image,
    const std::string & encoding,
    const sensor_msgs::msg::CameraInfo & depth_info,
    const sensor_msgs::msg::CameraInfo & color_info,
    const realsense2_camera_msgs::msg::Extrinsics & extrinsics,
    int color_width,
    int color_height,
    cv::Mat & color_depth,
    cv::Mat & distance_mask) const;

private:
  bool readDepthMeters(
    const cv::Mat & depth_image,
    const std::string & encoding,
    int row,
    int col,
    double & z_m) const;

  static void depthPixelToDepthCamera(
    int u,
    int v,
    double z,
    const sensor_msgs::msg::CameraInfo & depth_info,
    double & x_d,
    double & y_d,
    double & z_d);

  static void depthCameraToColorCamera(
    double x_d,
    double y_d,
    double z_d,
    const realsense2_camera_msgs::msg::Extrinsics & extrinsics,
    double & x_c,
    double & y_c,
    double & z_c);

  static bool colorCameraToPixel(
    double x_c,
    double y_c,
    double z_c,
    const sensor_msgs::msg::CameraInfo & color_info,
    int & u_c,
    int & v_c);

  DepthProjectorConfig config_;
};

}  // namespace weapon_tip_detector

#endif  // WEAPON_TIP_DETECTOR__DEPTH_PROJECTOR_HPP_