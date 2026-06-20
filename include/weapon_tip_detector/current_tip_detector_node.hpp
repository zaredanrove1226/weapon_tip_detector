#ifndef WEAPON_TIP_DETECTOR__CURRENT_TIP_DETECTOR_NODE_HPP_
#define WEAPON_TIP_DETECTOR__CURRENT_TIP_DETECTOR_NODE_HPP_

#include "weapon_tip_detector/depth_projector.hpp"
#include "weapon_tip_detector/detection_pipeline.hpp"
#include "weapon_tip_detector/detector_types.hpp"
#include "weapon_tip_detector/preview_debugger.hpp"
#include "weapon_tip_detector/palm_reference_detector.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "realsense2_camera_msgs/msg/extrinsics.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <memory>
#include <string>
#include <vector>

namespace weapon_tip_detector
{

class CurrentTipDetectorNode : public rclcpp::Node
{
  using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>;

public:
  CurrentTipDetectorNode();

private:
  void declareAndLoadParameters();

  void setupProfiles();

  void setupModules();

  void setupRosInterfaces();

  TipProfile declareProfile(
    const std::string & prefix,
    const TipProfile & defaults);

  bool isValidSlotId(int slot_id) const;

  std::string expectedTipType(int slot_id) const;

  TipProfile profileForType(const std::string & type) const;

  ProfileBundle makeProfileBundle() const;

  cv::Rect currentSlotRoi(int image_width, int image_height) const;

  cv::Rect currentDetectRoi(const cv::Rect & display_roi) const;

  void printCurrentRoiInfo(int image_width, int image_height) const;

  DepthProjectorConfig makeDepthProjectorConfig() const;

  DetectionPipelineConfig makeDetectionPipelineConfig() const;

  PreviewDebuggerConfig makePreviewDebuggerConfig() const;

  void currentSlotCallback(const std_msgs::msg::UInt8::SharedPtr msg);

  void depthCameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  void colorCameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  void extrinsicsCallback(
    const realsense2_camera_msgs::msg::Extrinsics::SharedPtr msg);

  void syncedImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg);

  void publishCurrentPresent(bool present);

  void logDetectionResult(const DetectionResult & result);

  std::string resolvePackagePath(const std::string & path) const;

  DetectionResult convertPalmReferenceResultToDetectionResult(
    const PalmReferenceResult & ref,
    const std::string & tip_type,
    const TipProfile & main_profile,
    const cv::Rect & display_roi,
    const cv::Rect & detect_roi,
    const cv::Mat & distance_mask) const;

private:
  int current_slot_id_{3};
  std::vector<std::string> slot_tip_types_;

  bool enable_slot_topic_{false};
  std::string current_slot_topic_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr current_slot_sub_;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr current_present_pub_;

  std::string depth_image_topic_;
  std::string rgb_image_topic_;
  std::string depth_camera_info_topic_;
  std::string color_camera_info_topic_;
  std::string extrinsics_topic_;

  int sync_queue_size_{10};

  message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_image_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_camera_info_sub_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr extrinsics_sensor_sub_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr extrinsics_transient_sub_;

  sensor_msgs::msg::CameraInfo::SharedPtr latest_depth_camera_info_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_color_camera_info_;
  realsense2_camera_msgs::msg::Extrinsics::SharedPtr latest_extrinsics_;

  double target_distance_{0.30};
  double distance_tolerance_{0.18};
  double valid_depth_min_{0.12};
  double valid_depth_max_{0.70};

  double slot_base_percentile_{0.75};
  int min_slot_base_valid_pixels_{50};
  double max_depth_delta_{0.18};

  int morph_open_size_{0};
  int morph_close_size_{5};
  int mask_dilate_size_{5};

  bool stable_enabled_{false};
  int stable_history_size_{5};
  int stable_accept_count_{3};

  double detect_roi_height_ratio_{1.0};

  bool enable_cv_preview_{true};
  double preview_scale_{0.8};
  int cv_wait_key_ms_{1};

  std::string display_mask_mode_{"all"};
  bool debug_show_roi_info_{true};

  std::vector<double> slot_roi_ratios_;

  TipProfile spear_profile_;
  bool spear_enable_dual_profile_{true};
  bool spear_stem_require_body_support_{true};
  double spear_stem_body_support_above_ratio_{0.60};
  double spear_stem_body_support_expand_x_ratio_{0.28};
  int spear_stem_body_support_min_dark_pixels_{140};
  double spear_stem_body_support_min_dark_ratio_{0.06};
  TipProfile spear_body_profile_;
  TipProfile spear_stem_profile_;

  TipProfile fist_profile_;
  bool enable_fist_dual_profile_{true};
  bool fist_stem_require_body_support_{true};
  double fist_stem_body_support_above_ratio_{0.55};
  double fist_stem_body_support_expand_x_ratio_{0.22};
  int fist_stem_body_support_min_dark_pixels_{120};
  double fist_stem_body_support_min_dark_ratio_{0.06};
  TipProfile fist_body_profile_;
  TipProfile fist_stem_profile_;

  TipProfile palm_profile_;

  bool palm_reference_enable_{false};
  PalmReferenceSlotParams palm_reference_slot3_params_;
  PalmReferenceSlotParams palm_reference_slot4_params_;
  
  std::unique_ptr<DepthProjector> depth_projector_;
  std::unique_ptr<DetectionPipeline> detection_pipeline_;
  std::unique_ptr<PreviewDebugger> preview_debugger_;

  // 新增 PalmReferenceDetector 成员
  PalmReferenceDetector palm_reference_detector_;
};

}  // namespace weapon_tip_detector

#endif  // WEAPON_TIP_DETECTOR__CURRENT_TIP_DETECTOR_NODE_HPP_