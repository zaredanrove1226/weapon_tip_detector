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

  slot_base_percentile_ = this->declare_parameter<double>("slot_base_percentile", 0.75);
  min_slot_base_valid_pixels_ = this->declare_parameter<int>("min_slot_base_valid_pixels", 50);
  max_depth_delta_ = this->declare_parameter<double>("max_depth_delta", 0.18);

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

  // --- 新增 PalmReference 参数 ---
  palm_reference_enable_ =
      this->declare_parameter<bool>("palm_reference_enable", false);

  // slot3
  {
      const std::string rgb_path =
          this->declare_parameter<std::string>("palm_reference_slot3_empty_rgb_path", "");
      const std::string depth_path =
          this->declare_parameter<std::string>("palm_reference_slot3_empty_depth_path", "");

      palm_reference_slot3_params_.empty_rgb_path = resolvePackagePath(rgb_path);
      palm_reference_slot3_params_.empty_depth_path = resolvePackagePath(depth_path);

      palm_reference_slot3_params_.dark_gray_threshold =
          this->declare_parameter<int>("palm_reference_slot3_dark_gray_threshold", 90);
      palm_reference_slot3_params_.rgb_diff_threshold =
          this->declare_parameter<int>("palm_reference_slot3_rgb_diff_threshold", 35);
      palm_reference_slot3_params_.depth_delta =
          this->declare_parameter<double>("palm_reference_slot3_depth_delta", 0.04);
      palm_reference_slot3_params_.allow_depth_hole =
          this->declare_parameter<bool>("palm_reference_slot3_allow_depth_hole", true);

      palm_reference_slot3_params_.min_area =
          this->declare_parameter<int>("palm_reference_slot3_min_area", 300);
      palm_reference_slot3_params_.min_width =
          this->declare_parameter<int>("palm_reference_slot3_min_width", 40);
      palm_reference_slot3_params_.min_height =
          this->declare_parameter<int>("palm_reference_slot3_min_height", 10);
      palm_reference_slot3_params_.min_aspect_ratio =
          this->declare_parameter<double>("palm_reference_slot3_min_aspect_ratio", 1.5);
      palm_reference_slot3_params_.min_fill_ratio =
          this->declare_parameter<double>("palm_reference_slot3_min_fill_ratio", 0.18);
  }

  // slot4
  {
      const std::string rgb_path =
          this->declare_parameter<std::string>("palm_reference_slot4_empty_rgb_path", "");
      const std::string depth_path =
          this->declare_parameter<std::string>("palm_reference_slot4_empty_depth_path", "");

      palm_reference_slot4_params_.empty_rgb_path = resolvePackagePath(rgb_path);
      palm_reference_slot4_params_.empty_depth_path = resolvePackagePath(depth_path);

      palm_reference_slot4_params_.dark_gray_threshold =
          this->declare_parameter<int>("palm_reference_slot4_dark_gray_threshold", 90);
      palm_reference_slot4_params_.rgb_diff_threshold =
          this->declare_parameter<int>("palm_reference_slot4_rgb_diff_threshold", 35);
      palm_reference_slot4_params_.depth_delta =
          this->declare_parameter<double>("palm_reference_slot4_depth_delta", 0.04);
      palm_reference_slot4_params_.allow_depth_hole =
          this->declare_parameter<bool>("palm_reference_slot4_allow_depth_hole", true);

      palm_reference_slot4_params_.min_area =
          this->declare_parameter<int>("palm_reference_slot4_min_area", 300);
      palm_reference_slot4_params_.min_width =
          this->declare_parameter<int>("palm_reference_slot4_min_width", 40);
      palm_reference_slot4_params_.min_height =
          this->declare_parameter<int>("palm_reference_slot4_min_height", 10);
      palm_reference_slot4_params_.min_aspect_ratio =
          this->declare_parameter<double>("palm_reference_slot4_min_aspect_ratio", 1.5);
      palm_reference_slot4_params_.min_fill_ratio =
          this->declare_parameter<double>("palm_reference_slot4_min_fill_ratio", 0.18);
  }

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

  spear_stem_require_body_support_ = this->declare_parameter<bool>(
    "spear_stem_require_body_support", true);
  spear_stem_body_support_above_ratio_ = this->declare_parameter<double>(
    "spear_stem_body_support_above_ratio", 0.60);
  spear_stem_body_support_expand_x_ratio_ = this->declare_parameter<double>(
    "spear_stem_body_support_expand_x_ratio", 0.28);
  spear_stem_body_support_min_dark_pixels_ = this->declare_parameter<int>(
    "spear_stem_body_support_min_dark_pixels", 140);
  spear_stem_body_support_min_dark_ratio_ = this->declare_parameter<double>(
    "spear_stem_body_support_min_dark_ratio", 0.06);

  spear_body_profile_ = declareProfile("spear_body", defaultSpearBodyProfile());
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
}

void CurrentTipDetectorNode::setupModules()
{
  depth_projector_ = std::make_unique<DepthProjector>(makeDepthProjectorConfig());
  detection_pipeline_ = std::make_unique<DetectionPipeline>(makeDetectionPipelineConfig());
  preview_debugger_ = std::make_unique<PreviewDebugger>(makePreviewDebuggerConfig());

  if (palm_reference_enable_) {
      bool ok3 = palm_reference_detector_.loadReference(3, palm_reference_slot3_params_);
      bool ok4 = palm_reference_detector_.loadReference(4, palm_reference_slot4_params_);

      RCLCPP_INFO(this->get_logger(), "Palm reference slot3 load: %s, rgb=%s, depth=%s",
                  ok3 ? "ok" : "failed",
                  palm_reference_slot3_params_.empty_rgb_path.c_str(),
                  palm_reference_slot3_params_.empty_depth_path.c_str());

      RCLCPP_INFO(this->get_logger(), "Palm reference slot4 load: %s, rgb=%s, depth=%s",
                  ok4 ? "ok" : "failed",
                  palm_reference_slot4_params_.empty_rgb_path.c_str(),
                  palm_reference_slot4_params_.empty_depth_path.c_str());
  } else {
      RCLCPP_INFO(this->get_logger(), "Palm reference diff disabled.");
  }

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

std::string CurrentTipDetectorNode::resolvePackagePath(
  const std::string & path) const
{
  if (path.empty()) {
    return "";
  }

  if (path.front() == '/') {
    return path;
  }

  try {
    const std::string share_dir =
      ament_index_cpp::get_package_share_directory("weapon_tip_detector");
    return share_dir + "/" + path;
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to resolve package share path for '%s': %s. Use raw path.",
      path.c_str(),
      e.what());
    return path;
  }
}

TipProfile CurrentTipDetectorNode::declareProfile(
  const std::string & prefix,
  const TipProfile & defaults)
{
  TipProfile p = defaults;

  p.candidate_mask_mode = this->declare_parameter<std::string>(
    prefix + "_candidate_mask_mode", p.candidate_mask_mode);
  p.min_depth_delta = this->declare_parameter<double>(
    prefix + "_min_depth_delta", p.min_depth_delta);
  p.min_component_area = this->declare_parameter<int>(
    prefix + "_min_component_area", p.min_component_area);
  p.min_candidate_score = this->declare_parameter<double>(
    prefix + "_min_candidate_score", p.min_candidate_score);
  p.max_component_area_ratio = this->declare_parameter<double>(
    prefix + "_max_component_area_ratio", p.max_component_area_ratio);
  p.require_depth_for_candidate = this->declare_parameter<bool>(
    prefix + "_require_depth_for_candidate", p.require_depth_for_candidate);

  p.enable_depth_too_far_veto = this->declare_parameter<bool>(
    prefix + "_enable_depth_too_far_veto", p.enable_depth_too_far_veto);
  p.depth_too_far_veto_min_count = this->declare_parameter<int>(
    prefix + "_depth_too_far_veto_min_count", p.depth_too_far_veto_min_count);
  p.depth_too_far_veto_max_delta = this->declare_parameter<double>(
    prefix + "_depth_too_far_veto_max_delta", p.depth_too_far_veto_max_delta);

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
  bundle.spear_body = spear_body_profile_;
  bundle.spear_stem = spear_stem_profile_;

  bundle.fist = fist_profile_;
  bundle.fist_body = fist_body_profile_;
  bundle.fist_stem = fist_stem_profile_;

  bundle.palm = palm_profile_;
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

  config.slot_base_percentile = slot_base_percentile_;
  config.min_slot_base_valid_pixels = min_slot_base_valid_pixels_;
  config.max_depth_delta = max_depth_delta_;

  config.morph_open_size = morph_open_size_;
  config.morph_close_size = morph_close_size_;
  config.mask_dilate_size = mask_dilate_size_;

  config.stable_enabled = stable_enabled_;
  config.stable_history_size = stable_history_size_;
  config.stable_accept_count = stable_accept_count_;

  config.enable_fist_dual_profile = enable_fist_dual_profile_;
  config.spear_enable_dual_profile = spear_enable_dual_profile_;

  config.fist_stem_body_support.require_support = fist_stem_require_body_support_;
  config.fist_stem_body_support.above_ratio = fist_stem_body_support_above_ratio_;
  config.fist_stem_body_support.expand_x_ratio = fist_stem_body_support_expand_x_ratio_;
  config.fist_stem_body_support.min_dark_pixels = fist_stem_body_support_min_dark_pixels_;
  config.fist_stem_body_support.min_dark_ratio = fist_stem_body_support_min_dark_ratio_;

  config.spear_stem_body_support.require_support = spear_stem_require_body_support_;
  config.spear_stem_body_support.above_ratio = spear_stem_body_support_above_ratio_;
  config.spear_stem_body_support.expand_x_ratio = spear_stem_body_support_expand_x_ratio_;
  config.spear_stem_body_support.min_dark_pixels = spear_stem_body_support_min_dark_pixels_;
  config.spear_stem_body_support.min_dark_ratio = spear_stem_body_support_min_dark_ratio_;

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

  DetectionResult result;

  const bool use_palm_reference =
      palm_reference_enable_ &&
      tip_type == "palm" &&
      (current_slot_id_ == 3 || current_slot_id_ == 4);

  if (use_palm_reference) {
      // 选择 slot 参数
      const PalmReferenceSlotParams & ref_params =
          (current_slot_id_ == 3) ? palm_reference_slot3_params_ : palm_reference_slot4_params_;

      // 先加载 reference，如果没加载就打印 warn
      if (!palm_reference_detector_.loadReference(current_slot_id_, ref_params)) {
          RCLCPP_WARN(this->get_logger(),
                      "Palm reference images not loaded for slot %d. Using normal detection.", 
                      current_slot_id_);
          result = detection_pipeline_->process(
              preview_bgr,
              color_depth,
              distance_mask,
              display_roi,
              detect_roi,
              tip_type,
              main_profile,
              makeProfileBundle());
      } else {
          // 执行 palm reference evaluate
          const PalmReferenceResult ref_result =
              palm_reference_detector_.evaluate(
                  current_slot_id_,
                  preview_bgr,
                  color_depth,
                  detect_roi);

          result = convertPalmReferenceResultToDetectionResult(
              ref_result,
              tip_type,
              main_profile,
              display_roi,
              detect_roi,
              distance_mask);
      }
  } else {
      // 普通 pipeline
      result = detection_pipeline_->process(
          preview_bgr,
          color_depth,
          distance_mask,
          display_roi,
          detect_roi,
          tip_type,
          main_profile,
          makeProfileBundle());
  }

  publishCurrentPresent(result.stable_present);

  if (preview_debugger_) {
    preview_debugger_->draw(preview_bgr, result, current_slot_id_);
    preview_debugger_->show(preview_bgr);
  }

  logDetectionResult(result);
}

DetectionResult CurrentTipDetectorNode::convertPalmReferenceResultToDetectionResult(
  const PalmReferenceResult & ref,
  const std::string & tip_type,
  const TipProfile & main_profile,
  const cv::Rect & display_roi,
  const cv::Rect & detect_roi,
  const cv::Mat & distance_mask) const
{
  DetectionResult result;

  TipProfile ref_profile = main_profile;
  ref_profile.candidate_mask_mode = "reference_diff";

  result.tip_type = tip_type;
  result.active_profile_name = "palm_reference_diff";
  result.active_profile = ref_profile;

  result.display_roi = display_roi;
  result.detect_roi = detect_roi;
  result.distance_mask = distance_mask;

  result.raw_present = ref.raw_present;
  result.stable_present = ref.raw_present;

  result.slot_base_depth = 0.0;
  result.slot_base_valid_count = 0;

  result.distance_pixels = 0;
  if (!distance_mask.empty()) {
    const cv::Rect safe_roi =
      detect_roi & cv::Rect(0, 0, distance_mask.cols, distance_mask.rows);
    if (safe_roi.area() > 0) {
      result.distance_pixels = cv::countNonZero(distance_mask(safe_roi));
    }
  }

  ProfileEvaluation ev;
  ev.name = "palm_reference_diff";
  ev.profile = ref_profile;

  ev.depth_candidate_mask = ref.depth_diff_mask;
  ev.dark_mask = ref.rgb_diff_mask;
  ev.candidate_mask = ref.candidate_mask;

  auto countInRoi = [&detect_roi](const cv::Mat & mask) -> int {
    if (mask.empty()) {
      return 0;
    }

    const cv::Rect safe_roi =
      detect_roi & cv::Rect(0, 0, mask.cols, mask.rows);

    if (safe_roi.area() <= 0) {
      return 0;
    }

    return cv::countNonZero(mask(safe_roi));
  };

  ev.depth_candidate_pixels = countInRoi(ref.depth_diff_mask);
  ev.dark_pixels = countInRoi(ref.rgb_diff_mask);
  ev.candidate_pixels = countInRoi(ref.candidate_mask);
  ev.raw_component_count = ref.raw_component_count;
  ev.accepted_candidate_count = ref.raw_present ? 1 : 0;

  Candidate best;
  best.exists = ref.best_bbox.area() > 0;
  best.accepted = ref.raw_present;
  best.bbox = ref.best_bbox;
  best.area = ref.best_area > 0 ? ref.best_area : ref.best_bbox.area();
  best.final_score = ref.final_score;
  best.rejected_reason = ref.rejected_reason;
  best.source_profile = "palm_reference_diff";

  if (ref.best_bbox.area() > 0 && detect_roi.area() > 0) {
    best.center = cv::Point2d(
      ref.best_bbox.x + ref.best_bbox.width * 0.5,
      ref.best_bbox.y + ref.best_bbox.height * 0.5);

    best.width_ratio =
      static_cast<double>(ref.best_bbox.width) /
      static_cast<double>(std::max(1, detect_roi.width));

    best.height_ratio =
      static_cast<double>(ref.best_bbox.height) /
      static_cast<double>(std::max(1, detect_roi.height));

    best.area_ratio =
      static_cast<double>(best.area) /
      static_cast<double>(std::max(1, detect_roi.area()));

    best.aspect_w_over_h =
      static_cast<double>(ref.best_bbox.width) /
      static_cast<double>(std::max(1, ref.best_bbox.height));

    best.fill_ratio =
      static_cast<double>(best.area) /
      static_cast<double>(
        std::max(1, ref.best_bbox.width * ref.best_bbox.height));

    best.center_x_ratio =
      (best.center.x - detect_roi.x) /
      static_cast<double>(std::max(1, detect_roi.width));

    best.center_y_ratio =
      (best.center.y - detect_roi.y) /
      static_cast<double>(std::max(1, detect_roi.height));

    best.depth_count = countInRoi(ref.depth_diff_mask);
    best.dark_count = countInRoi(ref.rgb_diff_mask);
    best.mask_count = countInRoi(ref.candidate_mask);

    if (best.mask_count > 0) {
      best.dark_ratio =
        static_cast<double>(best.dark_count) /
        static_cast<double>(best.mask_count);
    }
  }

  ev.best = best;

  result.main_eval = ev;

  return result;
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
    "slot=%d expected=%s profile=%s present=%s raw=%s stable=%s mode=%s cand_mode=%s dist=%d depth_cand=%d dark=%d cand=%d base=%.3f base_valid=%d comp=%d pass=%d best=%s accepted=%s reason=%s score=%.3f delta=%.3f area=%d bbox=(%d,%d,%d,%d) pos=(%.2f,%.2f) dark_ratio=%.2f depth_count=%d",
    current_slot_id_,
    result.tip_type.c_str(),
    result.active_profile_name.c_str(),
    result.stable_present ? "true" : "false",
    result.raw_present ? "true" : "false",
    stable_enabled_ ? "on" : "off",
    display_mask_mode_.c_str(),
    result.active_profile.candidate_mask_mode.c_str(),
    result.distance_pixels,
    ev.depth_candidate_pixels,
    ev.dark_pixels,
    ev.candidate_pixels,
    result.slot_base_depth,
    result.slot_base_valid_count,
    ev.raw_component_count,
    ev.accepted_candidate_count,
    best.exists ? "true" : "false",
    best.accepted ? "true" : "false",
    best.rejected_reason.c_str(),
    best.final_score,
    best.mean_depth_delta,
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