#include "weapon_tip_detector/current_tip_detector_node.hpp"

#include "weapon_tip_detector/detector_profiles.hpp"
#include "weapon_tip_detector/detector_utils.hpp"

#include "sensor_msgs/image_encodings.hpp"

#include "cv_bridge/cv_bridge.h"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace weapon_tip_detector
{

CurrentTipDetectorNode::CurrentTipDetectorNode()
: Node("current_tip_detector_node")
{
  declareAndLoadParameters();
  setupProfiles();
  setupModules();
  setupRosInterfaces();

  RCLCPP_INFO(this->get_logger(), "current_tip_detector_node started");
  RCLCPP_INFO(
    this->get_logger(),
    "slot=%d expected=%s target=%.3f tol=%.3f stable=%s display_mask_mode=%s",
    current_slot_id_,
    expectedTipType(current_slot_id_).c_str(),
    target_distance_,
    distance_tolerance_,
    stable_enabled_ ? "true" : "false",
    display_mask_mode_.c_str());

  printCurrentRoiInfo(1280, 720);
}

void CurrentTipDetectorNode::declareAndLoadParameters()
{
  current_slot_id_ = this->declare_parameter<int>("current_slot_id", 3);

  slot_tip_types_ = this->declare_parameter<std::vector<std::string>>(
    "slot_tip_types",
    std::vector<std::string>{"spear", "fist", "palm", "palm", "fist", "spear"});

  enable_slot_topic_ = this->declare_parameter<bool>("enable_slot_topic", false);
  current_slot_topic_ = this->declare_parameter<std::string>(
    "current_slot_topic",
    "/weapon_tip_detector/current_slot_id");

  depth_image_topic_ = this->declare_parameter<std::string>(
    "depth_image_topic",
    "/camera/camera/depth/image_rect_raw");

  rgb_image_topic_ = this->declare_parameter<std::string>(
    "rgb_image_topic",
    "/camera/camera/color/image_raw");

  depth_camera_info_topic_ = this->declare_parameter<std::string>(
    "depth_camera_info_topic",
    "/camera/camera/depth/camera_info");

  color_camera_info_topic_ = this->declare_parameter<std::string>(
    "color_camera_info_topic",
    "/camera/camera/color/camera_info");

  extrinsics_topic_ = this->declare_parameter<std::string>(
    "extrinsics_topic",
    "/camera/camera/extrinsics/depth_to_color");

  sync_queue_size_ = this->declare_parameter<int>("sync_queue_size", 10);

  target_distance_ = this->declare_parameter<double>("target_distance", 0.30);
  distance_tolerance_ = this->declare_parameter<double>("distance_tolerance", 0.18);
  valid_depth_min_ = this->declare_parameter<double>("valid_depth_min", 0.12);
  valid_depth_max_ = this->declare_parameter<double>("valid_depth_max", 0.70);

  background_percentile_ = this->declare_parameter<double>("background_percentile", 0.75);
  min_background_valid_pixels_ = this->declare_parameter<int>("min_background_valid_pixels", 50);
  foreground_max_depth_diff_ = this->declare_parameter<double>("foreground_max_depth_diff", 0.18);

  morph_open_size_ = this->declare_parameter<int>("morph_open_size", 0);
  morph_close_size_ = this->declare_parameter<int>("morph_close_size", 5);
  mask_dilate_size_ = this->declare_parameter<int>("mask_dilate_size", 5);

  stable_enabled_ = this->declare_parameter<bool>("stable_enabled", false);
  stable_history_size_ = this->declare_parameter<int>("stable_history_size", 5);
  stable_accept_count_ = this->declare_parameter<int>("stable_accept_count", 3);

  detect_roi_height_ratio_ = this->declare_parameter<double>("detect_roi_height_ratio", 1.0);

  enable_cv_preview_ = this->declare_parameter<bool>("enable_cv_preview", true);
  preview_scale_ = this->declare_parameter<double>("preview_scale", 0.8);
  cv_wait_key_ms_ = this->declare_parameter<int>("cv_wait_key_ms", 1);

  display_mask_mode_ = this->declare_parameter<std::string>("display_mask_mode", "all");
  debug_show_roi_info_ = this->declare_parameter<bool>("debug_show_roi_info", true);

  slot_roi_ratios_ = this->declare_parameter<std::vector<double>>(
    "slot_roi_ratios",
    std::vector<double>{
      0.41, 0.07, 0.18, 0.55,
      0.42, 0.18, 0.16, 0.38,
      0.39, 0.31, 0.22, 0.11,
      0.39, 0.31, 0.22, 0.11,
      0.42, 0.18, 0.16, 0.38,
      0.41, 0.07, 0.18, 0.55
    });

  if (slot_tip_types_.size() != 6) {
    RCLCPP_WARN(
      this->get_logger(),
      "slot_tip_types size is %zu, expected 6. Fallback to default mapping.",
      slot_tip_types_.size());
    slot_tip_types_ = {"spear", "fist", "palm", "palm", "fist", "spear"};
  }

  if (slot_roi_ratios_.size() != 24) {
    RCLCPP_WARN(
      this->get_logger(),
      "slot_roi_ratios size is %zu, expected 24. Fallback to centered ROI layout.",
      slot_roi_ratios_.size());
    slot_roi_ratios_ = {
      0.41, 0.07, 0.18, 0.55,
      0.42, 0.18, 0.16, 0.38,
      0.39, 0.31, 0.22, 0.11,
      0.39, 0.31, 0.22, 0.11,
      0.42, 0.18, 0.16, 0.38,
      0.41, 0.07, 0.18, 0.55
    };
  }
}

void CurrentTipDetectorNode::setupProfiles()
{
  spear_profile_ = declareProfile("spear", defaultSpearProfile());
  spear_enable_dual_profile_ = this->declare_parameter<bool>(
    "spear_enable_dual_profile", true);

  spear_stem_require_head_support_ = this->declare_parameter<bool>(
    "spear_stem_require_head_support", true);
  spear_stem_head_support_above_ratio_ = this->declare_parameter<double>(
    "spear_stem_head_support_above_ratio", 0.60);
  spear_stem_head_support_expand_x_ratio_ = this->declare_parameter<double>(
    "spear_stem_head_support_expand_x_ratio", 0.28);
  spear_stem_head_support_min_dark_pixels_ = this->declare_parameter<int>(
    "spear_stem_head_support_min_dark_pixels", 140);
  spear_stem_head_support_min_dark_ratio_ = this->declare_parameter<double>(
    "spear_stem_head_support_min_dark_ratio", 0.06);

  spear_head_profile_ = declareProfile("spear_head", defaultSpearHeadProfile());
  spear_stem_profile_ = declareProfile("spear_stem", defaultSpearStemProfile());

  fist_profile_ = declareProfile("fist", defaultFistProfile());
  enable_fist_dual_profile_ = this->declare_parameter<bool>(
    "enable_fist_dual_profile", true);

  fist_stem_require_body_support_ = this->declare_parameter<bool>(
    "fist_stem_require_body_support", true);
  fist_stem_body_support_above_ratio_ = this->declare_parameter<double>(
    "fist_stem_body_support_above_ratio", 0.55);
  fist_stem_body_support_expand_x_ratio_ = this->declare_parameter<double>(
    "fist_stem_body_support_expand_x_ratio", 0.22);
  fist_stem_body_support_min_dark_pixels_ = this->declare_parameter<int>(
    "fist_stem_body_support_min_dark_pixels", 120);
  fist_stem_body_support_min_dark_ratio_ = this->declare_parameter<double>(
    "fist_stem_body_support_min_dark_ratio", 0.06);

  fist_body_profile_ = declareProfile("fist_body", defaultFistBodyProfile());
  fist_stem_profile_ = declareProfile("fist_stem", defaultFistStemProfile());

  palm_profile_ = declareProfile("palm", defaultPalmProfile());
  enable_palm_dual_profile_ = this->declare_parameter<bool>(
    "enable_palm_dual_profile", true);
  palm_body_profile_ = declareProfile("palm_body", defaultPalmBodyProfile());
  palm_stem_profile_ = declareProfile("palm_stem", defaultPalmStemProfile());
}

void CurrentTipDetectorNode::setupModules()
{
  depth_projector_ = std::make_unique<DepthProjector>(makeDepthProjectorConfig());
  detection_pipeline_ = std::make_unique<DetectionPipeline>(makeDetectionPipelineConfig());
  preview_debugger_ = std::make_unique<PreviewDebugger>(makePreviewDebuggerConfig());

  preview_debugger_->createWindowIfEnabled();
}

void CurrentTipDetectorNode::setupRosInterfaces()
{
  if (enable_slot_topic_) {
    current_slot_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      current_slot_topic_,
      10,
      std::bind(&CurrentTipDetectorNode::currentSlotCallback, this, std::placeholders::_1));
  }

  current_present_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
    "/weapon_tip_detector/current_present",
    10);

  depth_image_sub_.subscribe(this, depth_image_topic_, rmw_qos_profile_sensor_data);
  rgb_image_sub_.subscribe(this, rgb_image_topic_, rmw_qos_profile_sensor_data);

  sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
    ApproxSyncPolicy(static_cast<uint32_t>(std::max(1, sync_queue_size_))),
    depth_image_sub_,
    rgb_image_sub_);

  sync_->registerCallback(
    std::bind(
      &CurrentTipDetectorNode::syncedImageCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  auto camera_info_qos = rclcpp::SensorDataQoS();
  auto extrinsics_sensor_qos = rclcpp::SensorDataQoS();
  auto extrinsics_transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  depth_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    depth_camera_info_topic_,
    camera_info_qos,
    std::bind(&CurrentTipDetectorNode::depthCameraInfoCallback, this, std::placeholders::_1));

  color_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    color_camera_info_topic_,
    camera_info_qos,
    std::bind(&CurrentTipDetectorNode::colorCameraInfoCallback, this, std::placeholders::_1));

  extrinsics_sensor_sub_ = this->create_subscription<realsense2_camera_msgs::msg::Extrinsics>(
    extrinsics_topic_,
    extrinsics_sensor_qos,
    std::bind(&CurrentTipDetectorNode::extrinsicsCallback, this, std::placeholders::_1));

  extrinsics_transient_sub_ = this->create_subscription<realsense2_camera_msgs::msg::Extrinsics>(
    extrinsics_topic_,
    extrinsics_transient_qos,
    std::bind(&CurrentTipDetectorNode::extrinsicsCallback, this, std::placeholders::_1));
}

TipProfile CurrentTipDetectorNode::declareProfile(
  const std::string & prefix,
  const TipProfile & defaults)
{
  TipProfile p = defaults;

  p.candidate_mask_mode = this->declare_parameter<std::string>(
    prefix + "_candidate_mask_mode", p.candidate_mask_mode);
  p.foreground_min_depth_diff = this->declare_parameter<double>(
    prefix + "_foreground_min_depth_diff", p.foreground_min_depth_diff);
  p.min_component_area = this->declare_parameter<int>(
    prefix + "_min_component_area", p.min_component_area);
  p.min_candidate_score = this->declare_parameter<double>(
    prefix + "_min_candidate_score", p.min_candidate_score);
  p.max_component_area_ratio = this->declare_parameter<double>(
    prefix + "_max_component_area_ratio", p.max_component_area_ratio);
  p.require_depth_for_candidate = this->declare_parameter<bool>(
    prefix + "_require_depth_for_candidate", p.require_depth_for_candidate);

  p.enable_depth_behind_veto = this->declare_parameter<bool>(
    prefix + "_enable_depth_behind_veto", p.enable_depth_behind_veto);
  p.depth_behind_veto_min_count = this->declare_parameter<int>(
    prefix + "_depth_behind_veto_min_count", p.depth_behind_veto_min_count);
  p.depth_behind_veto_max_diff = this->declare_parameter<double>(
    prefix + "_depth_behind_veto_max_diff", p.depth_behind_veto_max_diff);

  p.ideal_aspect_w_over_h = this->declare_parameter<double>(
    prefix + "_ideal_aspect_w_over_h", p.ideal_aspect_w_over_h);
  p.aspect_tolerance = this->declare_parameter<double>(
    prefix + "_aspect_tolerance", p.aspect_tolerance);
  p.ideal_width_ratio = this->declare_parameter<double>(
    prefix + "_ideal_width_ratio", p.ideal_width_ratio);
  p.width_tolerance = this->declare_parameter<double>(
    prefix + "_width_tolerance", p.width_tolerance);
  p.ideal_height_ratio = this->declare_parameter<double>(
    prefix + "_ideal_height_ratio", p.ideal_height_ratio);
  p.height_tolerance = this->declare_parameter<double>(
    prefix + "_height_tolerance", p.height_tolerance);

  p.min_width_ratio = this->declare_parameter<double>(
    prefix + "_min_width_ratio", p.min_width_ratio);
  p.min_height_ratio = this->declare_parameter<double>(
    prefix + "_min_height_ratio", p.min_height_ratio);
  p.min_fill_ratio = this->declare_parameter<double>(
    prefix + "_min_fill_ratio", p.min_fill_ratio);

  p.enable_position_score = this->declare_parameter<bool>(
    prefix + "_enable_position_score", p.enable_position_score);
  p.ideal_center_x_ratio = this->declare_parameter<double>(
    prefix + "_ideal_center_x_ratio", p.ideal_center_x_ratio);
  p.center_x_tolerance = this->declare_parameter<double>(
    prefix + "_center_x_tolerance", p.center_x_tolerance);
  p.ideal_center_y_ratio = this->declare_parameter<double>(
    prefix + "_ideal_center_y_ratio", p.ideal_center_y_ratio);
  p.center_y_tolerance = this->declare_parameter<double>(
    prefix + "_center_y_tolerance", p.center_y_tolerance);

  p.enable_roi_edge_suppression = this->declare_parameter<bool>(
    prefix + "_enable_roi_edge_suppression", p.enable_roi_edge_suppression);
  p.suppress_left_ratio = this->declare_parameter<double>(
    prefix + "_suppress_left_ratio", p.suppress_left_ratio);
  p.suppress_right_ratio = this->declare_parameter<double>(
    prefix + "_suppress_right_ratio", p.suppress_right_ratio);
  p.suppress_top_ratio = this->declare_parameter<double>(
    prefix + "_suppress_top_ratio", p.suppress_top_ratio);
  p.suppress_bottom_ratio = this->declare_parameter<double>(
    prefix + "_suppress_bottom_ratio", p.suppress_bottom_ratio);

  p.enable_ignore_mask = this->declare_parameter<bool>(
    prefix + "_enable_ignore_mask", p.enable_ignore_mask);

  const std::vector<double> ignore_rect_values =
    this->declare_parameter<std::vector<double>>(
      prefix + "_ignore_rects",
      std::vector<double>{});

  p.ignore_rects.clear();

  if (ignore_rect_values.size() % 4 != 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "%s_ignore_rects size is %zu, expected multiple of 4. Ignore mask disabled for this profile.",
      prefix.c_str(),
      ignore_rect_values.size());
    p.enable_ignore_mask = false;
  } else {
    for (size_t i = 0; i + 3 < ignore_rect_values.size(); i += 4) {
      p.ignore_rects.emplace_back(
        ignore_rect_values.at(i + 0),
        ignore_rect_values.at(i + 1),
        ignore_rect_values.at(i + 2),
        ignore_rect_values.at(i + 3));
    }
  }

  p.enable_palm_body_core_check = this->declare_parameter<bool>(
    prefix + "_enable_palm_body_core_check", p.enable_palm_body_core_check);

  const std::vector<double> palm_body_core_rect_values =
    this->declare_parameter<std::vector<double>>(
      prefix + "_palm_body_core_rect",
      std::vector<double>{
        p.palm_body_core_rect.x,
        p.palm_body_core_rect.y,
        p.palm_body_core_rect.width,
        p.palm_body_core_rect.height
      });

  if (palm_body_core_rect_values.size() == 4) {
    p.palm_body_core_rect = cv::Rect2d(
      palm_body_core_rect_values.at(0),
      palm_body_core_rect_values.at(1),
      palm_body_core_rect_values.at(2),
      palm_body_core_rect_values.at(3));
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "%s_palm_body_core_rect size is %zu, expected 4. Keep default body core rect.",
      prefix.c_str(),
      palm_body_core_rect_values.size());
  }

  p.palm_body_core_min_pixels = this->declare_parameter<int>(
    prefix + "_palm_body_core_min_pixels", p.palm_body_core_min_pixels);
  p.palm_body_core_min_density = this->declare_parameter<double>(
    prefix + "_palm_body_core_min_density", p.palm_body_core_min_density);
  p.palm_body_core_min_dark_ratio = this->declare_parameter<double>(
    prefix + "_palm_body_core_min_dark_ratio", p.palm_body_core_min_dark_ratio);

  p.enable_rgb_dark_filter = this->declare_parameter<bool>(
    prefix + "_enable_rgb_dark_filter", p.enable_rgb_dark_filter);
  p.rgb_dark_filter_mode = this->declare_parameter<std::string>(
    prefix + "_rgb_dark_filter_mode", p.rgb_dark_filter_mode);
  p.dark_gray_threshold = this->declare_parameter<int>(
    prefix + "_dark_gray_threshold", p.dark_gray_threshold);
  p.min_dark_ratio = this->declare_parameter<double>(
    prefix + "_min_dark_ratio", p.min_dark_ratio);

  p.shape_score_weight = this->declare_parameter<double>(
    prefix + "_shape_score_weight", p.shape_score_weight);
  p.depth_score_weight = this->declare_parameter<double>(
    prefix + "_depth_score_weight", p.depth_score_weight);
  p.area_score_weight = this->declare_parameter<double>(
    prefix + "_area_score_weight", p.area_score_weight);
  p.position_score_weight = this->declare_parameter<double>(
    prefix + "_position_score_weight", p.position_score_weight);
  p.dark_score_weight = this->declare_parameter<double>(
    prefix + "_dark_score_weight", p.dark_score_weight);

  return p;
}

bool CurrentTipDetectorNode::isValidSlotId(const int slot_id) const
{
  return slot_id >= 1 && slot_id <= 6;
}

std::string CurrentTipDetectorNode::expectedTipType(const int slot_id) const
{
  if (!isValidSlotId(slot_id)) {
    return "unknown";
  }

  return slot_tip_types_.at(static_cast<size_t>(slot_id - 1));
}

TipProfile CurrentTipDetectorNode::profileForType(const std::string & type) const
{
  if (type == "spear") {
    return spear_profile_;
  }

  if (type == "fist") {
    return fist_profile_;
  }

  if (type == "palm") {
    return palm_profile_;
  }

  return fist_profile_;
}

ProfileBundle CurrentTipDetectorNode::makeProfileBundle() const
{
  ProfileBundle bundle;
  bundle.spear = spear_profile_;
  bundle.spear_head = spear_head_profile_;
  bundle.spear_stem = spear_stem_profile_;

  bundle.fist = fist_profile_;
  bundle.fist_body = fist_body_profile_;
  bundle.fist_stem = fist_stem_profile_;

  bundle.palm = palm_profile_;
  bundle.palm_body = palm_body_profile_;
  bundle.palm_stem = palm_stem_profile_;
  return bundle;
}

cv::Rect CurrentTipDetectorNode::currentSlotRoi(
  const int image_width,
  const int image_height) const
{
  if (!isValidSlotId(current_slot_id_) || slot_roi_ratios_.size() != 24) {
    return cv::Rect();
  }

  const size_t base = static_cast<size_t>((current_slot_id_ - 1) * 4);

  cv::Rect roi(
    static_cast<int>(std::lround(slot_roi_ratios_.at(base + 0) * image_width)),
    static_cast<int>(std::lround(slot_roi_ratios_.at(base + 1) * image_height)),
    static_cast<int>(std::lround(slot_roi_ratios_.at(base + 2) * image_width)),
    static_cast<int>(std::lround(slot_roi_ratios_.at(base + 3) * image_height)));

  roi &= cv::Rect(0, 0, image_width, image_height);
  return roi;
}

cv::Rect CurrentTipDetectorNode::currentDetectRoi(const cv::Rect & display_roi) const
{
  if (display_roi.width <= 0 || display_roi.height <= 0) {
    return cv::Rect();
  }

  const double ratio = std::clamp(detect_roi_height_ratio_, 0.10, 1.00);

  cv::Rect detect_roi(
    display_roi.x,
    display_roi.y,
    display_roi.width,
    static_cast<int>(std::lround(static_cast<double>(display_roi.height) * ratio)));

  detect_roi &= display_roi;
  return detect_roi;
}

void CurrentTipDetectorNode::printCurrentRoiInfo(
  const int image_width,
  const int image_height) const
{
  const cv::Rect display_roi = currentSlotRoi(image_width, image_height);
  const cv::Rect detect_roi = currentDetectRoi(display_roi);

  RCLCPP_INFO(
    this->get_logger(),
    "ROI check for %dx%d: display_roi=(x=%d,y=%d,w=%d,h=%d), detect_roi=(x=%d,y=%d,w=%d,h=%d)",
    image_width,
    image_height,
    display_roi.x,
    display_roi.y,
    display_roi.width,
    display_roi.height,
    detect_roi.x,
    detect_roi.y,
    detect_roi.width,
    detect_roi.height);
}

DepthProjectorConfig CurrentTipDetectorNode::makeDepthProjectorConfig() const
{
  DepthProjectorConfig config;
  config.target_distance = target_distance_;
  config.distance_tolerance = distance_tolerance_;
  config.valid_depth_min = valid_depth_min_;
  config.valid_depth_max = valid_depth_max_;
  config.mask_dilate_size = mask_dilate_size_;
  return config;
}

DetectionPipelineConfig CurrentTipDetectorNode::makeDetectionPipelineConfig() const
{
  DetectionPipelineConfig config;
  config.target_distance = target_distance_;
  config.valid_depth_min = valid_depth_min_;
  config.valid_depth_max = valid_depth_max_;

  config.background_percentile = background_percentile_;
  config.min_background_valid_pixels = min_background_valid_pixels_;
  config.foreground_max_depth_diff = foreground_max_depth_diff_;

  config.morph_open_size = morph_open_size_;
  config.morph_close_size = morph_close_size_;
  config.mask_dilate_size = mask_dilate_size_;

  config.stable_enabled = stable_enabled_;
  config.stable_history_size = stable_history_size_;
  config.stable_accept_count = stable_accept_count_;

  config.enable_fist_dual_profile = enable_fist_dual_profile_;
  config.spear_enable_dual_profile = spear_enable_dual_profile_;
  config.enable_palm_dual_profile = enable_palm_dual_profile_;

  config.fist_stem_body_support.require_support = fist_stem_require_body_support_;
  config.fist_stem_body_support.above_ratio = fist_stem_body_support_above_ratio_;
  config.fist_stem_body_support.expand_x_ratio = fist_stem_body_support_expand_x_ratio_;
  config.fist_stem_body_support.min_dark_pixels = fist_stem_body_support_min_dark_pixels_;
  config.fist_stem_body_support.min_dark_ratio = fist_stem_body_support_min_dark_ratio_;

  config.spear_stem_head_support.require_support = spear_stem_require_head_support_;
  config.spear_stem_head_support.above_ratio = spear_stem_head_support_above_ratio_;
  config.spear_stem_head_support.expand_x_ratio = spear_stem_head_support_expand_x_ratio_;
  config.spear_stem_head_support.min_dark_pixels = spear_stem_head_support_min_dark_pixels_;
  config.spear_stem_head_support.min_dark_ratio = spear_stem_head_support_min_dark_ratio_;

  return config;
}

PreviewDebuggerConfig CurrentTipDetectorNode::makePreviewDebuggerConfig() const
{
  PreviewDebuggerConfig config;
  config.enable_cv_preview = enable_cv_preview_;
  config.preview_scale = preview_scale_;
  config.cv_wait_key_ms = cv_wait_key_ms_;
  config.display_mask_mode = display_mask_mode_;
  config.debug_show_roi_info = debug_show_roi_info_;
  return config;
}

void CurrentTipDetectorNode::currentSlotCallback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  const int new_slot_id = static_cast<int>(msg->data);

  if (!isValidSlotId(new_slot_id)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Received invalid current_slot_id from topic: %d. Valid range is 1~6.",
      new_slot_id);
    return;
  }

  if (new_slot_id == current_slot_id_) {
    return;
  }

  current_slot_id_ = new_slot_id;

  if (detection_pipeline_) {
    detection_pipeline_->resetStableHistory();
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Updated current_slot_id: %d, expected_tip_type: %s",
    current_slot_id_,
    expectedTipType(current_slot_id_).c_str());
}

void CurrentTipDetectorNode::depthCameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  latest_depth_camera_info_ = msg;
}

void CurrentTipDetectorNode::colorCameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  latest_color_camera_info_ = msg;
}

void CurrentTipDetectorNode::extrinsicsCallback(
  const realsense2_camera_msgs::msg::Extrinsics::SharedPtr msg)
{
  latest_extrinsics_ = msg;
}

void CurrentTipDetectorNode::syncedImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg)
{
  if (!depth_msg || !rgb_msg) {
    return;
  }

  if (!latest_depth_camera_info_ || !latest_color_camera_info_ || !latest_extrinsics_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "Waiting for calibration data: depth_info=%s color_info=%s extrinsics=%s",
      latest_depth_camera_info_ ? "ok" : "missing",
      latest_color_camera_info_ ? "ok" : "missing",
      latest_extrinsics_ ? "ok" : "missing");
    return;
  }

  if (!validIntrinsics(*latest_depth_camera_info_) ||
      !validIntrinsics(*latest_color_camera_info_))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "Invalid camera intrinsics.");
    return;
  }

  if (depth_msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
      depth_msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "Unsupported depth encoding: %s",
      depth_msg->encoding.c_str());
    return;
  }

  cv_bridge::CvImageConstPtr depth_cv_ptr;
  cv_bridge::CvImagePtr rgb_cv_ptr;

  try {
    depth_cv_ptr = cv_bridge::toCvShare(depth_msg);
    rgb_cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "cv_bridge conversion failed: %s",
      ex.what());
    return;
  }

  cv::Mat preview_bgr = rgb_cv_ptr->image;

  const cv::Rect display_roi = currentSlotRoi(preview_bgr.cols, preview_bgr.rows);
  const cv::Rect detect_roi = currentDetectRoi(display_roi);

  if (!isValidSlotId(current_slot_id_) ||
      display_roi.width <= 0 || display_roi.height <= 0 ||
      detect_roi.width <= 0 || detect_roi.height <= 0)
  {
    if (preview_debugger_) {
      preview_debugger_->drawInvalidRoi(preview_bgr);
      preview_debugger_->show(preview_bgr);
    }
    return;
  }

  cv::Mat color_depth;
  cv::Mat distance_mask;

  const bool projected = depth_projector_->projectDepthToColor(
    depth_cv_ptr->image,
    depth_msg->encoding,
    *latest_depth_camera_info_,
    *latest_color_camera_info_,
    *latest_extrinsics_,
    preview_bgr.cols,
    preview_bgr.rows,
    color_depth,
    distance_mask);

  if (!projected) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "Depth projection failed.");
  }

  const std::string tip_type = expectedTipType(current_slot_id_);
  const TipProfile main_profile = profileForType(tip_type);

  DetectionResult result = detection_pipeline_->process(
    preview_bgr,
    color_depth,
    distance_mask,
    display_roi,
    detect_roi,
    tip_type,
    main_profile,
    makeProfileBundle());

  publishCurrentPresent(result.stable_present);

  if (preview_debugger_) {
    preview_debugger_->draw(preview_bgr, result, current_slot_id_);
    preview_debugger_->show(preview_bgr);
  }

  logDetectionResult(result);
}

void CurrentTipDetectorNode::publishCurrentPresent(const bool present)
{
  if (!current_present_pub_) {
    return;
  }

  std_msgs::msg::UInt8 msg;
  msg.data = present ? 1 : 0;
  current_present_pub_->publish(msg);
}

void CurrentTipDetectorNode::logDetectionResult(
  const DetectionResult & result)
{
  const ProfileEvaluation & ev = result.main_eval;
  const Candidate & best = ev.best;

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "slot=%d expected=%s profile=%s present=%s raw=%s stable=%s mode=%s cand_mode=%s dist=%d fg=%d dark=%d cand=%d bg=%.3f bg_valid=%d comp=%d pass=%d best=%s accepted=%s reason=%s score=%.3f diff=%.3f area=%d bbox=(%d,%d,%d,%d) pos=(%.2f,%.2f) dark_ratio=%.2f depth_count=%d",
    current_slot_id_,
    result.tip_type.c_str(),
    result.active_profile_name.c_str(),
    result.stable_present ? "true" : "false",
    result.raw_present ? "true" : "false",
    stable_enabled_ ? "on" : "off",
    display_mask_mode_.c_str(),
    result.active_profile.candidate_mask_mode.c_str(),
    result.distance_pixels,
    ev.foreground_pixels,
    ev.dark_pixels,
    ev.candidate_pixels,
    result.background_depth,
    result.background_valid_count,
    ev.raw_component_count,
    ev.accepted_candidate_count,
    best.exists ? "true" : "false",
    best.accepted ? "true" : "false",
    best.rejected_reason.c_str(),
    best.final_score,
    best.mean_depth_diff,
    best.area,
    best.bbox.x,
    best.bbox.y,
    best.bbox.width,
    best.bbox.height,
    best.center_x_ratio,
    best.center_y_ratio,
    best.dark_ratio,
    best.depth_count);
}

}  // namespace weapon_tip_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<weapon_tip_detector::CurrentTipDetectorNode>());
  rclcpp::shutdown();
  return 0;
}