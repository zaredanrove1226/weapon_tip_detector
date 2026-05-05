#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "realsense2_camera_msgs/msg/extrinsics.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "cv_bridge/cv_bridge.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{

inline double clamp01(const double v)
{
  return std::clamp(v, 0.0, 1.0);
}

inline bool validIntrinsics(const sensor_msgs::msg::CameraInfo & info)
{
  return info.k[0] > 1e-6 && info.k[4] > 1e-6;
}

inline void depthPixelToDepthCamera(
  int u, int v, double z,
  const sensor_msgs::msg::CameraInfo & depth_info,
  double & x_d, double & y_d, double & z_d)
{
  const double fx = depth_info.k[0];
  const double fy = depth_info.k[4];
  const double cx = depth_info.k[2];
  const double cy = depth_info.k[5];

  x_d = (static_cast<double>(u) - cx) * z / fx;
  y_d = (static_cast<double>(v) - cy) * z / fy;
  z_d = z;
}

inline void depthCameraToColorCamera(
  double x_d, double y_d, double z_d,
  const realsense2_camera_msgs::msg::Extrinsics & extr,
  double & x_c, double & y_c, double & z_c)
{
  const auto & r = extr.rotation;
  const auto & t = extr.translation;

  // RealSense extrinsics rotation is column-major 3x3.
  x_c = r[0] * x_d + r[3] * y_d + r[6] * z_d + t[0];
  y_c = r[1] * x_d + r[4] * y_d + r[7] * z_d + t[1];
  z_c = r[2] * x_d + r[5] * y_d + r[8] * z_d + t[2];
}

inline bool colorCameraToPixel(
  double x_c, double y_c, double z_c,
  const sensor_msgs::msg::CameraInfo & color_info,
  int & u_c, int & v_c)
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

inline double percentile(std::vector<float> values, const double p)
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

inline double closenessScore(
  const double value,
  const double ideal,
  const double tolerance)
{
  const double tol = std::max(1e-6, tolerance);
  return clamp01(1.0 - std::abs(value - ideal) / tol);
}

}  // namespace

namespace weapon_tip_detector
{

struct TipProfile
{
  std::string type{"unknown"};

  // Which mask is used to generate connected-component candidates:
  // foreground, dark, foreground_or_dark, foreground_and_dark, distance, distance_or_dark.
  std::string candidate_mask_mode{"foreground"};

  double foreground_min_depth_diff{0.02};
  int min_component_area{20};
  double min_candidate_score{0.30};
  double max_component_area_ratio{0.60};
  bool require_depth_for_candidate{true};

  double ideal_aspect_w_over_h{1.0};
  double aspect_tolerance{1.0};
  double ideal_width_ratio{0.40};
  double width_tolerance{0.50};
  double ideal_height_ratio{0.40};
  double height_tolerance{0.50};

  double min_width_ratio{0.0};
  double min_height_ratio{0.0};
  double min_fill_ratio{0.0};

  bool enable_position_score{true};
  double ideal_center_x_ratio{0.50};
  double center_x_tolerance{0.50};
  double ideal_center_y_ratio{0.50};
  double center_y_tolerance{0.50};

  bool enable_roi_edge_suppression{true};
  double suppress_left_ratio{0.00};
  double suppress_right_ratio{0.00};
  double suppress_top_ratio{0.00};
  double suppress_bottom_ratio{0.12};

  bool enable_rgb_dark_filter{true};
  std::string rgb_dark_filter_mode{"score"};  // off, score, filter
  int dark_gray_threshold{95};
  double min_dark_ratio{0.20};

  double shape_score_weight{0.35};
  double depth_score_weight{0.25};
  double area_score_weight{0.15};
  double position_score_weight{0.15};
  double dark_score_weight{0.10};
};

struct Candidate
{
  bool exists{false};
  bool accepted{false};

  int label{0};
  int area{0};
  cv::Rect bbox;
  cv::Point2d center;

  int depth_count{0};
  int dark_count{0};
  int mask_count{0};

  double mean_depth{0.0};
  double mean_depth_diff{0.0};

  double area_ratio{0.0};
  double width_ratio{0.0};
  double height_ratio{0.0};
  double aspect_w_over_h{0.0};
  double fill_ratio{0.0};
  double center_x_ratio{0.0};
  double center_y_ratio{0.0};
  double dark_ratio{0.0};

  double shape_score{0.0};
  double area_score{0.0};
  double depth_score{0.0};
  double position_score{0.0};
  double dark_score{0.0};
  double final_score{0.0};

  std::string rejected_reason{"none"};
  std::string source_profile{"main"};
};

struct ProfileEvaluation
{
  std::string name{"main"};
  TipProfile profile;

  cv::Mat foreground_mask;
  cv::Mat dark_mask;
  cv::Mat candidate_mask;

  int foreground_pixels{0};
  int dark_pixels{0};
  int candidate_pixels{0};
  int raw_component_count{0};
  int accepted_candidate_count{0};

  Candidate best;

  bool rawPresent() const
  {
    return best.exists && best.accepted;
  }
};

class CurrentTipDetectorNode : public rclcpp::Node
{
  using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>;

public:
  CurrentTipDetectorNode()
  : Node("current_tip_detector_node")
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

    // display_mask_mode: distance, foreground, dark, candidate, combined, all.
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

    spear_profile_ = declareProfile("spear", defaultSpearProfile());
    spear_enable_dual_profile_ = this->declare_parameter<bool>("spear_enable_dual_profile", true);

    // spear_stem 不能单独确认 spear。
    // 它只作为“连接杆/底座”辅助证据：
    // 如果只有一根细杆，没有上方矛尖主体暗色支撑，则拒绝 stem。
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
    enable_fist_dual_profile_ = this->declare_parameter<bool>("enable_fist_dual_profile", true);

    // fist_stem 不能单独确认 fist。
    // 它只作为“连接杆/下半部分”辅助证据：
    // 如果只有一根细杆，没有上方拳体暗色支撑，则拒绝 stem。
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

    if (enable_cv_preview_) {
      cv::namedWindow("current_tip_detector_preview", cv::WINDOW_NORMAL);
    }

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

private:
  TipProfile defaultSpearProfile() const
  {
    TipProfile p;
    p.type = "spear";
    p.candidate_mask_mode = "foreground_or_dark";
    p.foreground_min_depth_diff = 0.008;
    p.min_component_area = 14;
    p.min_candidate_score = 0.22;
    p.max_component_area_ratio = 0.40;
    p.require_depth_for_candidate = false;
    p.ideal_aspect_w_over_h = 0.35;
    p.aspect_tolerance = 1.00;
    p.ideal_width_ratio = 0.35;
    p.width_tolerance = 0.70;
    p.ideal_height_ratio = 0.58;
    p.height_tolerance = 0.80;
    p.enable_position_score = true;
    p.ideal_center_x_ratio = 0.50;
    p.center_x_tolerance = 0.45;
    p.ideal_center_y_ratio = 0.45;
    p.center_y_tolerance = 0.65;
    p.enable_roi_edge_suppression = true;
    p.suppress_bottom_ratio = 0.25;
    p.enable_rgb_dark_filter = true;
    p.rgb_dark_filter_mode = "score";
    p.dark_gray_threshold = 85;
    p.min_dark_ratio = 0.25;
    p.shape_score_weight = 0.30;
    p.depth_score_weight = 0.20;
    p.area_score_weight = 0.10;
    p.position_score_weight = 0.15;
    p.dark_score_weight = 0.25;
    return p;
  }

  TipProfile defaultSpearHeadProfile() const
  {
    TipProfile p = defaultSpearProfile();
    p.type = "spear_head";
    p.candidate_mask_mode = "foreground_or_dark";
    p.foreground_min_depth_diff = 0.008;
    p.min_component_area = 25;
    p.min_candidate_score = 0.30;
    p.max_component_area_ratio = 0.35;
    p.require_depth_for_candidate = false;

    // 矛尖主体：偏高、偏窄、暗色面积明显。
    p.ideal_aspect_w_over_h = 0.45;
    p.aspect_tolerance = 0.90;
    p.ideal_width_ratio = 0.42;
    p.width_tolerance = 0.65;
    p.ideal_height_ratio = 0.55;
    p.height_tolerance = 0.70;

    p.enable_position_score = true;
    p.ideal_center_x_ratio = 0.50;
    p.center_x_tolerance = 0.50;
    p.ideal_center_y_ratio = 0.46;
    p.center_y_tolerance = 0.60;

    p.enable_roi_edge_suppression = true;
    p.suppress_left_ratio = 0.00;
    p.suppress_right_ratio = 0.00;
    p.suppress_top_ratio = 0.00;
    p.suppress_bottom_ratio = 0.22;

    p.enable_rgb_dark_filter = true;
    p.rgb_dark_filter_mode = "score";
    p.dark_gray_threshold = 90;
    p.min_dark_ratio = 0.14;

    p.shape_score_weight = 0.34;
    p.depth_score_weight = 0.18;
    p.area_score_weight = 0.14;
    p.position_score_weight = 0.14;
    p.dark_score_weight = 0.20;

    p.min_width_ratio = 0.18;
    p.min_height_ratio = 0.25;
    p.min_fill_ratio = 0.18;
    return p;
  }

  TipProfile defaultSpearStemProfile() const
  {
    TipProfile p = defaultSpearProfile();
    p.type = "spear_stem";
    p.candidate_mask_mode = "foreground_and_dark";
    p.foreground_min_depth_diff = 0.010;
    p.min_component_area = 10;
    p.min_candidate_score = 0.34;
    p.max_component_area_ratio = 0.13;
    p.require_depth_for_candidate = true;

    // 连接杆/底座：面积小、偏竖直、深度结构更明显。
    p.ideal_aspect_w_over_h = 0.35;
    p.aspect_tolerance = 0.70;
    p.ideal_width_ratio = 0.22;
    p.width_tolerance = 0.38;
    p.ideal_height_ratio = 0.45;
    p.height_tolerance = 0.58;

    p.enable_position_score = true;
    p.ideal_center_x_ratio = 0.50;
    p.center_x_tolerance = 0.45;
    p.ideal_center_y_ratio = 0.64;
    p.center_y_tolerance = 0.40;

    p.enable_roi_edge_suppression = true;
    p.suppress_left_ratio = 0.00;
    p.suppress_right_ratio = 0.00;
    p.suppress_top_ratio = 0.00;
    p.suppress_bottom_ratio = 0.20;

    p.enable_rgb_dark_filter = true;
    p.rgb_dark_filter_mode = "score";
    p.dark_gray_threshold = 95;
    p.min_dark_ratio = 0.12;

    p.shape_score_weight = 0.30;
    p.depth_score_weight = 0.25;
    p.area_score_weight = 0.12;
    p.position_score_weight = 0.18;
    p.dark_score_weight = 0.15;

    p.min_width_ratio = 0.04;
    p.min_height_ratio = 0.12;
    p.min_fill_ratio = 0.10;
    return p;
  }

  TipProfile defaultFistProfile() const
  {
    TipProfile p;
    p.type = "fist";
    p.candidate_mask_mode = "foreground_or_dark";
    p.foreground_min_depth_diff = 0.010;
    p.min_component_area = 18;
    p.min_candidate_score = 0.25;
    p.max_component_area_ratio = 0.40;
    p.require_depth_for_candidate = false;
    p.ideal_aspect_w_over_h = 0.85;
    p.aspect_tolerance = 1.10;
    p.ideal_width_ratio = 0.45;
    p.width_tolerance = 0.70;
    p.ideal_height_ratio = 0.48;
    p.height_tolerance = 0.70;
    p.enable_position_score = true;
    p.ideal_center_x_ratio = 0.50;
    p.center_x_tolerance = 0.45;
    p.ideal_center_y_ratio = 0.50;
    p.center_y_tolerance = 0.60;
    p.enable_roi_edge_suppression = true;
    p.suppress_bottom_ratio = 0.18;
    p.enable_rgb_dark_filter = true;
    p.rgb_dark_filter_mode = "score";
    p.dark_gray_threshold = 85;
    p.min_dark_ratio = 0.20;
    p.shape_score_weight = 0.32;
    p.depth_score_weight = 0.20;
    p.area_score_weight = 0.13;
    p.position_score_weight = 0.15;
    p.dark_score_weight = 0.20;
    return p;
  }

  TipProfile defaultFistBodyProfile() const
  {
    TipProfile p = defaultFistProfile();
    p.type = "fist_body";
    p.candidate_mask_mode = "foreground_or_dark";
    p.foreground_min_depth_diff = 0.010;
    p.min_component_area = 22;
    p.min_candidate_score = 0.28;
    p.max_component_area_ratio = 0.34;
    p.require_depth_for_candidate = false;

    // 拳体主体：块状、暗色、面积较大。
    p.ideal_aspect_w_over_h = 0.82;
    p.aspect_tolerance = 1.20;
    p.ideal_width_ratio = 0.46;
    p.width_tolerance = 0.75;
    p.ideal_height_ratio = 0.48;
    p.height_tolerance = 0.75;

    p.enable_position_score = true;
    p.ideal_center_x_ratio = 0.50;
    p.center_x_tolerance = 0.55;
    p.ideal_center_y_ratio = 0.44;
    p.center_y_tolerance = 0.55;

    p.enable_roi_edge_suppression = true;
    p.suppress_bottom_ratio = 0.16;

    p.enable_rgb_dark_filter = true;
    p.rgb_dark_filter_mode = "score";
    p.dark_gray_threshold = 90;
    p.min_dark_ratio = 0.18;

    p.shape_score_weight = 0.32;
    p.depth_score_weight = 0.16;
    p.area_score_weight = 0.14;
    p.position_score_weight = 0.13;
    p.dark_score_weight = 0.25;
    return p;
  }

  TipProfile defaultFistStemProfile() const
  {
    TipProfile p = defaultFistProfile();
    p.type = "fist_stem";
    p.candidate_mask_mode = "foreground_or_dark";
    p.foreground_min_depth_diff = 0.008;
    p.min_component_area = 8;
    p.min_candidate_score = 0.30;
    p.max_component_area_ratio = 0.16;
    p.require_depth_for_candidate = false;

    // 连接杆/下半部分：更细、更竖，面积小；作为 fist 的辅助确认。
    p.ideal_aspect_w_over_h = 0.35;
    p.aspect_tolerance = 0.75;
    p.ideal_width_ratio = 0.20;
    p.width_tolerance = 0.35;
    p.ideal_height_ratio = 0.42;
    p.height_tolerance = 0.60;

    p.enable_position_score = true;
    p.ideal_center_x_ratio = 0.50;
    p.center_x_tolerance = 0.55;
    p.ideal_center_y_ratio = 0.68;
    p.center_y_tolerance = 0.40;

    p.enable_roi_edge_suppression = true;
    p.suppress_bottom_ratio = 0.10;

    p.enable_rgb_dark_filter = true;
    p.rgb_dark_filter_mode = "score";
    p.dark_gray_threshold = 95;
    p.min_dark_ratio = 0.10;

    p.shape_score_weight = 0.28;
    p.depth_score_weight = 0.22;
    p.area_score_weight = 0.12;
    p.position_score_weight = 0.20;
    p.dark_score_weight = 0.18;
    return p;
  }

  TipProfile defaultPalmProfile() const
  {
    TipProfile p;
    p.type = "palm";
    p.candidate_mask_mode = "foreground_and_dark";
    p.foreground_min_depth_diff = 0.015;
    p.min_component_area = 45;
    p.min_candidate_score = 0.38;
    p.max_component_area_ratio = 0.30;
    p.require_depth_for_candidate = true;
    p.ideal_aspect_w_over_h = 2.00;
    p.aspect_tolerance = 2.20;
    p.ideal_width_ratio = 0.55;
    p.width_tolerance = 0.70;
    p.ideal_height_ratio = 0.30;
    p.height_tolerance = 0.60;
    p.enable_position_score = true;
    p.ideal_center_x_ratio = 0.50;
    p.center_x_tolerance = 0.50;
    p.ideal_center_y_ratio = 0.45;
    p.center_y_tolerance = 0.45;
    p.enable_roi_edge_suppression = true;
    p.suppress_bottom_ratio = 0.25;
    p.enable_rgb_dark_filter = true;
    p.rgb_dark_filter_mode = "score";
    p.dark_gray_threshold = 95;
    p.min_dark_ratio = 0.20;
    p.shape_score_weight = 0.35;
    p.depth_score_weight = 0.30;
    p.area_score_weight = 0.15;
    p.position_score_weight = 0.15;
    p.dark_score_weight = 0.05;
    return p;
  }

  TipProfile declareProfile(const std::string & prefix, const TipProfile & defaults)
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

    p.min_width_ratio = this->declare_parameter<double>(prefix + "_min_width_ratio", p.min_width_ratio);
    p.min_height_ratio = this->declare_parameter<double>(prefix + "_min_height_ratio", p.min_height_ratio);
    p.min_fill_ratio = this->declare_parameter<double>(prefix + "_min_fill_ratio", p.min_fill_ratio);

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

  bool isValidSlotId(const int slot_id) const
  {
    return slot_id >= 1 && slot_id <= 6;
  }

  std::string expectedTipType(const int slot_id) const
  {
    if (!isValidSlotId(slot_id)) {
      return "unknown";
    }
    return slot_tip_types_.at(static_cast<size_t>(slot_id - 1));
  }

  TipProfile profileForType(const std::string & type) const
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

  cv::Rect currentSlotRoi(const int image_width, const int image_height) const
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

  cv::Rect currentDetectRoi(const cv::Rect & display_roi) const
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

  void printCurrentRoiInfo(const int image_width, const int image_height) const
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

  void currentSlotCallback(const std_msgs::msg::UInt8::SharedPtr msg)
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
    stable_history_.clear();

    RCLCPP_INFO(
      this->get_logger(),
      "Updated current_slot_id: %d, expected_tip_type: %s",
      current_slot_id_,
      expectedTipType(current_slot_id_).c_str());
  }

  void depthCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    latest_depth_camera_info_ = msg;
  }

  void colorCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    latest_color_camera_info_ = msg;
  }

  void extrinsicsCallback(const realsense2_camera_msgs::msg::Extrinsics::SharedPtr msg)
  {
    latest_extrinsics_ = msg;
  }

  bool readDepthMeters(
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

  void projectDepthToColor(
    const cv::Mat & depth_image,
    const std::string & encoding,
    const int color_width,
    const int color_height,
    cv::Mat & color_depth,
    cv::Mat & distance_mask)
  {
    color_depth = cv::Mat(
      color_height,
      color_width,
      CV_32FC1,
      cv::Scalar(std::numeric_limits<float>::quiet_NaN()));

    distance_mask = cv::Mat::zeros(color_height, color_width, CV_8UC1);

    const double lower = std::max(valid_depth_min_, target_distance_ - distance_tolerance_);
    const double upper = std::min(valid_depth_max_, target_distance_ + distance_tolerance_);

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

        depthPixelToDepthCamera(u, v, z_m, *latest_depth_camera_info_, x_d, y_d, z_d);

        double x_c = 0.0;
        double y_c = 0.0;
        double z_c = 0.0;

        depthCameraToColorCamera(x_d, y_d, z_d, *latest_extrinsics_, x_c, y_c, z_c);

        int u_c = 0;
        int v_c = 0;

        if (!colorCameraToPixel(x_c, y_c, z_c, *latest_color_camera_info_, u_c, v_c)) {
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

    if (cv::countNonZero(distance_mask) > 0 && mask_dilate_size_ > 1) {
      const int k = mask_dilate_size_;
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
      cv::dilate(distance_mask, distance_mask, kernel);
    }
  }

  double estimateBackgroundDepth(
    const cv::Mat & color_depth,
    const cv::Rect & roi,
    int & valid_count) const
  {
    std::vector<float> depths;
    depths.reserve(static_cast<size_t>(std::max(0, roi.area())));

    valid_count = 0;

    for (int y = roi.y; y < roi.y + roi.height; ++y) {
      for (int x = roi.x; x < roi.x + roi.width; ++x) {
        const float z = color_depth.at<float>(y, x);

        if (!std::isfinite(z)) {
          continue;
        }

        if (z < valid_depth_min_ || z > valid_depth_max_) {
          continue;
        }

        depths.push_back(z);
      }
    }

    valid_count = static_cast<int>(depths.size());

    if (valid_count < min_background_valid_pixels_) {
      return target_distance_;
    }

    const double bg = percentile(depths, background_percentile_);

    if (!std::isfinite(bg)) {
      return target_distance_;
    }

    return bg;
  }

  cv::Mat buildForegroundMask(
    const cv::Mat & color_depth,
    const cv::Rect & detect_roi,
    const double background_depth,
    const TipProfile & profile) const
  {
    cv::Mat fg_mask = cv::Mat::zeros(color_depth.size(), CV_8UC1);

    for (int y = detect_roi.y; y < detect_roi.y + detect_roi.height; ++y) {
      for (int x = detect_roi.x; x < detect_roi.x + detect_roi.width; ++x) {
        const float z_f = color_depth.at<float>(y, x);

        if (!std::isfinite(z_f)) {
          continue;
        }

        const double z = static_cast<double>(z_f);

        if (z < valid_depth_min_ || z > valid_depth_max_) {
          continue;
        }

        const double diff = background_depth - z;

        if (diff >= profile.foreground_min_depth_diff &&
            diff <= foreground_max_depth_diff_)
        {
          fg_mask.at<uint8_t>(y, x) = 255;
        }
      }
    }

    if (cv::countNonZero(fg_mask) == 0) {
      return fg_mask;
    }

    if (mask_dilate_size_ > 1) {
      const int k = mask_dilate_size_;
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
      cv::dilate(fg_mask, fg_mask, kernel);
    }

    if (morph_close_size_ > 1) {
      const int k = morph_close_size_;
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
      cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_CLOSE, kernel);
    }

    if (morph_open_size_ > 1) {
      const int k = morph_open_size_;
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
      cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_OPEN, kernel);
    }

    return fg_mask;
  }

  cv::Mat buildDarkMask(
    const cv::Mat & bgr_image,
    const cv::Rect & detect_roi,
    const TipProfile & profile) const
  {
    cv::Mat dark_mask = cv::Mat::zeros(bgr_image.size(), CV_8UC1);

    if (!profile.enable_rgb_dark_filter || profile.rgb_dark_filter_mode == "off") {
      return dark_mask;
    }

    cv::Mat gray;
    cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat local_dark;
    cv::threshold(
      gray(detect_roi),
      local_dark,
      static_cast<double>(std::clamp(profile.dark_gray_threshold, 0, 255)),
      255.0,
      cv::THRESH_BINARY_INV);

    local_dark.copyTo(dark_mask(detect_roi));

    return dark_mask;
  }

  cv::Mat buildCandidateMask(
    const cv::Mat & distance_mask,
    const cv::Mat & foreground_mask,
    const cv::Mat & dark_mask,
    const cv::Rect & detect_roi,
    const TipProfile & profile) const
  {
    cv::Mat candidate_mask = cv::Mat::zeros(foreground_mask.size(), CV_8UC1);

    if (profile.candidate_mask_mode == "dark") {
      dark_mask(detect_roi).copyTo(candidate_mask(detect_roi));
    } else if (profile.candidate_mask_mode == "foreground_or_dark") {
      cv::Mat combined;
      cv::bitwise_or(foreground_mask, dark_mask, combined);
      combined(detect_roi).copyTo(candidate_mask(detect_roi));
    } else if (profile.candidate_mask_mode == "foreground_and_dark") {
      cv::Mat combined;
      cv::bitwise_and(foreground_mask, dark_mask, combined);
      combined(detect_roi).copyTo(candidate_mask(detect_roi));
    } else if (profile.candidate_mask_mode == "distance") {
      distance_mask(detect_roi).copyTo(candidate_mask(detect_roi));
    } else if (profile.candidate_mask_mode == "distance_or_dark") {
      cv::Mat combined;
      cv::bitwise_or(distance_mask, dark_mask, combined);
      combined(detect_roi).copyTo(candidate_mask(detect_roi));
    } else {
      foreground_mask(detect_roi).copyTo(candidate_mask(detect_roi));
    }

    return candidate_mask;
  }

  bool isSuppressedByRoiEdge(const Candidate & c, const TipProfile & profile) const
  {
    if (!profile.enable_roi_edge_suppression) {
      return false;
    }

    if (c.center_x_ratio < profile.suppress_left_ratio) {
      return true;
    }

    if (c.center_x_ratio > 1.0 - profile.suppress_right_ratio) {
      return true;
    }

    if (c.center_y_ratio < profile.suppress_top_ratio) {
      return true;
    }

    if (c.center_y_ratio > 1.0 - profile.suppress_bottom_ratio) {
      return true;
    }

    return false;
  }

  Candidate scoreCandidate(
    const int label,
    const cv::Mat & labels,
    const cv::Mat & stats,
    const cv::Mat & centroids,
    const cv::Mat & color_depth_roi,
    const cv::Mat & dark_mask_roi,
    const cv::Rect & local_detect_roi,
    const double background_depth,
    const TipProfile & profile) const
  {
    Candidate c;
    c.exists = true;
    c.label = label;

    const int x = stats.at<int>(label, cv::CC_STAT_LEFT);
    const int y = stats.at<int>(label, cv::CC_STAT_TOP);
    const int w = stats.at<int>(label, cv::CC_STAT_WIDTH);
    const int h = stats.at<int>(label, cv::CC_STAT_HEIGHT);
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);

    c.area = area;
    c.bbox = cv::Rect(x, y, w, h);
    c.center = cv::Point2d(
      centroids.at<double>(label, 0),
      centroids.at<double>(label, 1));

    if (w <= 0 || h <= 0 || area <= 0) {
      c.rejected_reason = "empty";
      return c;
    }

    const double detect_area = static_cast<double>(std::max(1, local_detect_roi.area()));

    c.area_ratio = static_cast<double>(area) / detect_area;
    c.width_ratio = static_cast<double>(w) / static_cast<double>(std::max(1, local_detect_roi.width));
    c.height_ratio = static_cast<double>(h) / static_cast<double>(std::max(1, local_detect_roi.height));
    c.aspect_w_over_h = static_cast<double>(w) / static_cast<double>(std::max(1, h));
    c.fill_ratio = static_cast<double>(area) / static_cast<double>(std::max(1, w * h));

    c.center_x_ratio = c.center.x / static_cast<double>(std::max(1, local_detect_roi.width));
    c.center_y_ratio = c.center.y / static_cast<double>(std::max(1, local_detect_roi.height));

    double depth_sum = 0.0;
    double diff_sum = 0.0;

    for (int yy = y; yy < y + h; ++yy) {
      for (int xx = x; xx < x + w; ++xx) {
        if (labels.at<int>(yy, xx) != label) {
          continue;
        }

        ++c.mask_count;

        if (dark_mask_roi.at<uint8_t>(yy, xx) > 0) {
          ++c.dark_count;
        }

        const float z_f = color_depth_roi.at<float>(yy, xx);
        if (!std::isfinite(z_f)) {
          continue;
        }

        const double z = static_cast<double>(z_f);
        const double diff = background_depth - z;

        depth_sum += z;
        diff_sum += diff;
        ++c.depth_count;
      }
    }

    c.dark_ratio =
      c.mask_count > 0 ? static_cast<double>(c.dark_count) / static_cast<double>(c.mask_count) : 0.0;

    if (c.depth_count > 0) {
      c.mean_depth = depth_sum / static_cast<double>(c.depth_count);
      c.mean_depth_diff = diff_sum / static_cast<double>(c.depth_count);
    } else {
      c.mean_depth = 0.0;
      c.mean_depth_diff = 0.0;
    }

    const double aspect_score = closenessScore(
      c.aspect_w_over_h,
      profile.ideal_aspect_w_over_h,
      profile.aspect_tolerance);

    const double width_score = closenessScore(
      c.width_ratio,
      profile.ideal_width_ratio,
      profile.width_tolerance);

    const double height_score = closenessScore(
      c.height_ratio,
      profile.ideal_height_ratio,
      profile.height_tolerance);

    const double fill_score = clamp01(c.fill_ratio / 0.60);

    c.shape_score =
      0.40 * aspect_score +
      0.25 * width_score +
      0.25 * height_score +
      0.10 * fill_score;

    c.area_score = clamp01(
      static_cast<double>(area) /
      static_cast<double>(std::max(1, profile.min_component_area * 5)));

    if (c.depth_count > 0) {
      c.depth_score = clamp01(
        (c.mean_depth_diff - profile.foreground_min_depth_diff) /
        std::max(0.005, profile.foreground_min_depth_diff * 5.0));
    } else {
      c.depth_score = 0.0;
    }

    if (profile.enable_position_score) {
      const double x_score = closenessScore(
        c.center_x_ratio,
        profile.ideal_center_x_ratio,
        profile.center_x_tolerance);
      const double y_score = closenessScore(
        c.center_y_ratio,
        profile.ideal_center_y_ratio,
        profile.center_y_tolerance);
      c.position_score = 0.5 * x_score + 0.5 * y_score;
    } else {
      c.position_score = 1.0;
    }

    if (profile.enable_rgb_dark_filter && profile.rgb_dark_filter_mode != "off") {
      c.dark_score = clamp01(c.dark_ratio / std::max(1e-6, profile.min_dark_ratio));
    } else {
      c.dark_score = 1.0;
    }

    double weight_sum =
      std::max(0.0, profile.shape_score_weight) +
      std::max(0.0, profile.area_score_weight) +
      (profile.require_depth_for_candidate || c.depth_count > 0 ? std::max(0.0, profile.depth_score_weight) : 0.0) +
      (profile.enable_position_score ? std::max(0.0, profile.position_score_weight) : 0.0) +
      (profile.enable_rgb_dark_filter && profile.rgb_dark_filter_mode != "off" ?
        std::max(0.0, profile.dark_score_weight) : 0.0);

    if (weight_sum <= 1e-6) {
      weight_sum = 1.0;
    }

    c.final_score =
      (
        std::max(0.0, profile.shape_score_weight) * c.shape_score +
        std::max(0.0, profile.area_score_weight) * c.area_score +
        (profile.require_depth_for_candidate || c.depth_count > 0 ?
          std::max(0.0, profile.depth_score_weight) * c.depth_score : 0.0) +
        (profile.enable_position_score ?
          std::max(0.0, profile.position_score_weight) * c.position_score : 0.0) +
        (profile.enable_rgb_dark_filter && profile.rgb_dark_filter_mode != "off" ?
          std::max(0.0, profile.dark_score_weight) * c.dark_score : 0.0)
      ) / weight_sum;

    if (area < profile.min_component_area) {
      c.rejected_reason = "area_small";
      return c;
    }

    if (c.width_ratio < profile.min_width_ratio) {
      c.rejected_reason = "width_small";
      return c;
    }

    if (c.height_ratio < profile.min_height_ratio) {
      c.rejected_reason = "height_small";
      return c;
    }

    if (c.fill_ratio < profile.min_fill_ratio) {
      c.rejected_reason = "fill_small";
      return c;
    }

    if (c.area_ratio > profile.max_component_area_ratio) {
      c.rejected_reason = "area_large";
      return c;
    }

    if (profile.require_depth_for_candidate && c.depth_count <= 0) {
      c.rejected_reason = "no_depth";
      return c;
    }

    if (profile.require_depth_for_candidate &&
        c.mean_depth_diff < profile.foreground_min_depth_diff)
    {
      c.rejected_reason = "diff_small";
      return c;
    }

    if (isSuppressedByRoiEdge(c, profile)) {
      c.rejected_reason = "roi_edge";
      return c;
    }

    if (profile.enable_rgb_dark_filter &&
        profile.rgb_dark_filter_mode == "filter" &&
        c.dark_ratio < profile.min_dark_ratio)
    {
      c.rejected_reason = "dark_low";
      return c;
    }

    if (c.final_score < profile.min_candidate_score) {
      c.rejected_reason = "score_low";
      return c;
    }

    c.accepted = true;
    c.rejected_reason = "accepted";
    return c;
  }

  Candidate findBestCandidate(
    const cv::Mat & candidate_mask,
    const cv::Mat & color_depth,
    const cv::Mat & dark_mask,
    const cv::Rect & detect_roi,
    const double background_depth,
    const TipProfile & profile,
    int & raw_component_count,
    int & accepted_candidate_count) const
  {
    Candidate best;
    raw_component_count = 0;
    accepted_candidate_count = 0;

    if (detect_roi.width <= 0 || detect_roi.height <= 0) {
      return best;
    }

    cv::Mat roi_mask = candidate_mask(detect_roi).clone();

    if (cv::countNonZero(roi_mask) == 0) {
      return best;
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;

    const int n_labels = cv::connectedComponentsWithStats(
      roi_mask,
      labels,
      stats,
      centroids,
      8,
      CV_32S);

    raw_component_count = std::max(0, n_labels - 1);

    for (int label = 1; label < n_labels; ++label) {
      Candidate local = scoreCandidate(
        label,
        labels,
        stats,
        centroids,
        color_depth(detect_roi),
        dark_mask(detect_roi),
        cv::Rect(0, 0, detect_roi.width, detect_roi.height),
        background_depth,
        profile);

      if (!local.exists) {
        continue;
      }

      local.bbox.x += detect_roi.x;
      local.bbox.y += detect_roi.y;
      local.center.x += detect_roi.x;
      local.center.y += detect_roi.y;

      if (local.accepted) {
        ++accepted_candidate_count;
      }

      if (!best.exists || local.final_score > best.final_score) {
        best = local;
      }
    }

    return best;
  }

  bool updateStableResult(const bool raw_present)
  {
    if (!stable_enabled_) {
      stable_history_.clear();
      stable_history_.push_back(raw_present);
      return raw_present;
    }

    const int history_size = std::max(1, stable_history_size_);
    const int accept_count = std::clamp(stable_accept_count_, 1, history_size);

    stable_history_.push_back(raw_present);

    while (static_cast<int>(stable_history_.size()) > history_size) {
      stable_history_.pop_front();
    }

    int true_count = 0;
    for (const bool v : stable_history_) {
      if (v) {
        ++true_count;
      }
    }

    return true_count >= accept_count;
  }

  void drawText(
    cv::Mat & image,
    const std::string & text,
    const int y,
    const cv::Scalar & color,
    const double scale = 0.50,
    const int thickness = 2) const
  {
    cv::putText(
      image,
      text,
      cv::Point(18, y),
      cv::FONT_HERSHEY_SIMPLEX,
      scale,
      color,
      thickness);
  }

  cv::Mat selectSingleDebugMask(
    const cv::Mat & distance_mask,
    const cv::Mat & foreground_mask,
    const cv::Mat & dark_mask,
    const cv::Mat & candidate_mask,
    const cv::Rect & detect_roi) const
  {
    cv::Mat debug_mask = cv::Mat::zeros(distance_mask.size(), CV_8UC1);

    if (display_mask_mode_ == "distance") {
      distance_mask(detect_roi).copyTo(debug_mask(detect_roi));
      return debug_mask;
    }

    if (display_mask_mode_ == "foreground") {
      foreground_mask(detect_roi).copyTo(debug_mask(detect_roi));
      return debug_mask;
    }

    if (display_mask_mode_ == "dark") {
      dark_mask(detect_roi).copyTo(debug_mask(detect_roi));
      return debug_mask;
    }

    if (display_mask_mode_ == "candidate") {
      candidate_mask(detect_roi).copyTo(debug_mask(detect_roi));
      return debug_mask;
    }

    cv::Mat combined;
    cv::bitwise_or(distance_mask, foreground_mask, combined);
    cv::bitwise_or(combined, dark_mask, combined);
    cv::bitwise_or(combined, candidate_mask, combined);
    combined(detect_roi).copyTo(debug_mask(detect_roi));
    return debug_mask;
  }

  void overlayMaskColor(
    cv::Mat & overlay,
    const cv::Mat & mask,
    const cv::Scalar & color,
    const cv::Rect & roi) const
  {
    if (cv::countNonZero(mask(roi)) <= 0) {
      return;
    }

    cv::Mat colored = cv::Mat::zeros(overlay.size(), overlay.type());
    colored.setTo(color, mask);
    cv::addWeighted(colored, 0.35, overlay, 1.0, 0.0, overlay);
  }

  ProfileEvaluation evaluateProfile(
    const std::string & name,
    const TipProfile & profile,
    const cv::Mat & preview_bgr,
    const cv::Mat & color_depth,
    const cv::Mat & distance_mask,
    const cv::Rect & detect_roi,
    const double background_depth) const
  {
    ProfileEvaluation ev;
    ev.name = name;
    ev.profile = profile;

    ev.foreground_mask = buildForegroundMask(color_depth, detect_roi, background_depth, profile);
    ev.dark_mask = buildDarkMask(preview_bgr, detect_roi, profile);
    ev.candidate_mask = buildCandidateMask(
      distance_mask, ev.foreground_mask, ev.dark_mask, detect_roi, profile);

    ev.foreground_pixels = cv::countNonZero(ev.foreground_mask(detect_roi));
    ev.dark_pixels = cv::countNonZero(ev.dark_mask(detect_roi));
    ev.candidate_pixels = cv::countNonZero(ev.candidate_mask(detect_roi));

    ev.best = findBestCandidate(
      ev.candidate_mask,
      color_depth,
      ev.dark_mask,
      detect_roi,
      background_depth,
      profile,
      ev.raw_component_count,
      ev.accepted_candidate_count);
    ev.best.source_profile = name;

    return ev;
  }

  static Candidate chooseBestCandidate(const Candidate & a, const Candidate & b)
  {
    if (a.exists && a.accepted && !(b.exists && b.accepted)) {
      return a;
    }
    if (b.exists && b.accepted && !(a.exists && a.accepted)) {
      return b;
    }
    if (a.exists && b.exists) {
      return a.final_score >= b.final_score ? a : b;
    }
    if (a.exists) {
      return a;
    }
    return b;
  }

  bool hasUpperDarkSupport(
    const Candidate & lower_candidate,
    const cv::Mat & upper_dark_mask,
    const cv::Rect & detect_roi,
    const double above_ratio,
    const double expand_x_ratio,
    const int min_dark_pixels,
    const double min_dark_ratio,
    int & support_dark_pixels,
    double & support_dark_ratio) const
  {
    support_dark_pixels = 0;
    support_dark_ratio = 0.0;

    if (!lower_candidate.exists ||
        lower_candidate.bbox.width <= 0 ||
        lower_candidate.bbox.height <= 0)
    {
      return false;
    }

    // 支撑区：lower/stem 候选的上方、横向稍微放宽的一块区域。
    // fist 用它检查拳体暗色支撑；spear 用它检查矛尖主体暗色支撑。
    const int expand_x = static_cast<int>(
      std::lround(static_cast<double>(detect_roi.width) * expand_x_ratio));
    const int above_h = static_cast<int>(
      std::lround(static_cast<double>(detect_roi.height) * above_ratio));

    const int lower_cx = lower_candidate.bbox.x + lower_candidate.bbox.width / 2;
    const int x0 = lower_cx - expand_x;
    const int x1 = lower_cx + expand_x;
    const int y1 = lower_candidate.bbox.y;
    const int y0 = y1 - above_h;

    cv::Rect support_roi(
      x0,
      y0,
      std::max(1, x1 - x0),
      std::max(1, y1 - y0));

    support_roi &= detect_roi;

    if (support_roi.width <= 0 || support_roi.height <= 0) {
      return false;
    }

    support_dark_pixels = cv::countNonZero(upper_dark_mask(support_roi));
    support_dark_ratio = static_cast<double>(support_dark_pixels) /
      static_cast<double>(std::max(1, support_roi.area()));

    return support_dark_pixels >= min_dark_pixels && support_dark_ratio >= min_dark_ratio;
  }

  bool hasFistStemBodySupport(
    const Candidate & stem_candidate,
    const cv::Mat & body_dark_mask,
    const cv::Rect & detect_roi,
    int & support_dark_pixels,
    double & support_dark_ratio) const
  {
    if (!fist_stem_require_body_support_) {
      support_dark_pixels = 0;
      support_dark_ratio = 0.0;
      return true;
    }

    return hasUpperDarkSupport(
      stem_candidate,
      body_dark_mask,
      detect_roi,
      fist_stem_body_support_above_ratio_,
      fist_stem_body_support_expand_x_ratio_,
      fist_stem_body_support_min_dark_pixels_,
      fist_stem_body_support_min_dark_ratio_,
      support_dark_pixels,
      support_dark_ratio);
  }

  static cv::Mat orMaskSafe(const cv::Mat & a, const cv::Mat & b)
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

  void syncedImageCallback(
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
      drawText(preview_bgr, "Invalid current_slot_id or ROI", 40, cv::Scalar(0, 0, 255), 0.8);
      showPreview(preview_bgr);
      return;
    }

    const std::string tip_type = expectedTipType(current_slot_id_);
    TipProfile profile = profileForType(tip_type);
    std::string active_profile_name = tip_type;

    cv::Mat color_depth;
    cv::Mat distance_mask;
    projectDepthToColor(
      depth_cv_ptr->image,
      depth_msg->encoding,
      preview_bgr.cols,
      preview_bgr.rows,
      color_depth,
      distance_mask);

    int background_valid_count = 0;
    const double background_depth = estimateBackgroundDepth(
      color_depth,
      display_roi,
      background_valid_count);

    ProfileEvaluation main_eval;

    // 注意：raw_present 不能只依赖最后被拿来显示的 best。
    // 对 fist 双 profile，真正的判定规则是：
    // 1) fist_body accepted  => 直接确认 fist；
    // 2) fist_stem accepted  => 必须有 body support 才确认 fist；
    // 3) 两者都不通过       => raw NO，但仍显示分数最高的 rejected candidate 方便调试。
    bool main_eval_raw_present = false;

    if (tip_type == "fist" && enable_fist_dual_profile_) {
      ProfileEvaluation body_eval = evaluateProfile(
        "fist_body", fist_body_profile_, preview_bgr, color_depth, distance_mask,
        detect_roi, background_depth);
      ProfileEvaluation stem_eval = evaluateProfile(
        "fist_stem", fist_stem_profile_, preview_bgr, color_depth, distance_mask,
        detect_roi, background_depth);

      int stem_support_dark_pixels = 0;
      double stem_support_dark_ratio = 0.0;
      const bool stem_has_body_support = hasFistStemBodySupport(
        stem_eval.best,
        body_eval.dark_mask,
        detect_roi,
        stem_support_dark_pixels,
        stem_support_dark_ratio);

      const bool body_ok = body_eval.best.exists && body_eval.best.accepted;
      bool stem_ok = stem_eval.best.exists && stem_eval.best.accepted;

      // 关键规则：fist_body 可以单独确认 fist；
      // fist_stem 不能单独确认 fist，必须在上方/附近有拳体暗色支撑。
      if (stem_ok && !stem_has_body_support) {
        stem_ok = false;
        stem_eval.best.accepted = false;
        stem_eval.best.rejected_reason = "stem_no_body";
        stem_eval.accepted_candidate_count = 0;
      }

      main_eval.name = "fist_dual";
      main_eval.foreground_mask = orMaskSafe(body_eval.foreground_mask, stem_eval.foreground_mask);
      main_eval.dark_mask = orMaskSafe(body_eval.dark_mask, stem_eval.dark_mask);
      main_eval.candidate_mask = orMaskSafe(body_eval.candidate_mask, stem_eval.candidate_mask);
      main_eval.foreground_pixels = cv::countNonZero(main_eval.foreground_mask(detect_roi));
      main_eval.dark_pixels = cv::countNonZero(main_eval.dark_mask(detect_roi));
      main_eval.candidate_pixels = cv::countNonZero(main_eval.candidate_mask(detect_roi));
      main_eval.raw_component_count = body_eval.raw_component_count + stem_eval.raw_component_count;
      main_eval.accepted_candidate_count = body_eval.accepted_candidate_count + stem_eval.accepted_candidate_count;

      if (body_ok) {
        // 最重要的修复：只要 fist_body 通过，raw_present 必须是 true。
        main_eval.best = body_eval.best;
        main_eval.profile = body_eval.profile;
        profile = fist_body_profile_;
        active_profile_name = "fist_body";
        main_eval_raw_present = true;
      } else if (stem_ok) {
        main_eval.best = stem_eval.best;
        main_eval.profile = stem_eval.profile;
        profile = fist_stem_profile_;
        active_profile_name = "fist_stem";
        main_eval_raw_present = true;
      } else {
        // 没有 accepted 时，仍显示最值得看的候选，方便继续判断是分数/形状/深度的问题。
        main_eval.best = chooseBestCandidate(body_eval.best, stem_eval.best);
        if (main_eval.best.source_profile == "fist_stem") {
          main_eval.profile = stem_eval.profile;
          profile = fist_stem_profile_;
          active_profile_name = stem_has_body_support ? "fist_stem" : "fist_stem_no_body";
        } else {
          main_eval.profile = body_eval.profile;
          profile = fist_body_profile_;
          active_profile_name = "fist_body";
        }
        main_eval_raw_present = false;
      }
    } else if (tip_type == "spear" && spear_enable_dual_profile_) {
      ProfileEvaluation head_eval = evaluateProfile(
        "spear_head", spear_head_profile_, preview_bgr, color_depth, distance_mask,
        detect_roi, background_depth);
      ProfileEvaluation stem_eval = evaluateProfile(
        "spear_stem", spear_stem_profile_, preview_bgr, color_depth, distance_mask,
        detect_roi, background_depth);

      int stem_support_dark_pixels = 0;
      double stem_support_dark_ratio = 0.0;
      bool stem_has_head_support = true;
      if (spear_stem_require_head_support_) {
        stem_has_head_support = hasUpperDarkSupport(
          stem_eval.best,
          head_eval.dark_mask,
          detect_roi,
          spear_stem_head_support_above_ratio_,
          spear_stem_head_support_expand_x_ratio_,
          spear_stem_head_support_min_dark_pixels_,
          spear_stem_head_support_min_dark_ratio_,
          stem_support_dark_pixels,
          stem_support_dark_ratio);
      }

      const bool head_ok = head_eval.best.exists && head_eval.best.accepted;
      bool stem_ok = stem_eval.best.exists && stem_eval.best.accepted;

      // 关键规则：spear_head 可以单独确认 spear；
      // spear_stem 不能单独确认 spear，必须在上方/附近有矛尖主体暗色支撑。
      if (stem_ok && !stem_has_head_support) {
        stem_ok = false;
        stem_eval.best.accepted = false;
        stem_eval.best.rejected_reason = "stem_no_head";
        stem_eval.accepted_candidate_count = 0;
      }

      main_eval.name = "spear_dual";
      main_eval.foreground_mask = orMaskSafe(head_eval.foreground_mask, stem_eval.foreground_mask);
      main_eval.dark_mask = orMaskSafe(head_eval.dark_mask, stem_eval.dark_mask);
      main_eval.candidate_mask = orMaskSafe(head_eval.candidate_mask, stem_eval.candidate_mask);
      main_eval.foreground_pixels = cv::countNonZero(main_eval.foreground_mask(detect_roi));
      main_eval.dark_pixels = cv::countNonZero(main_eval.dark_mask(detect_roi));
      main_eval.candidate_pixels = cv::countNonZero(main_eval.candidate_mask(detect_roi));
      main_eval.raw_component_count = head_eval.raw_component_count + stem_eval.raw_component_count;
      main_eval.accepted_candidate_count = head_eval.accepted_candidate_count + stem_eval.accepted_candidate_count;

      if (head_ok) {
        // 只要 spear_head 通过，raw_present 必须是 true。
        main_eval.best = head_eval.best;
        main_eval.profile = head_eval.profile;
        profile = spear_head_profile_;
        active_profile_name = "spear_head";
        main_eval_raw_present = true;
      } else if (stem_ok) {
        main_eval.best = stem_eval.best;
        main_eval.profile = stem_eval.profile;
        profile = spear_stem_profile_;
        active_profile_name = "spear_stem";
        main_eval_raw_present = true;
      } else {
        // 没有 accepted 时，仍显示最值得看的候选，方便继续判断是分数/形状/深度的问题。
        main_eval.best = chooseBestCandidate(head_eval.best, stem_eval.best);
        if (main_eval.best.source_profile == "spear_stem") {
          main_eval.profile = stem_eval.profile;
          profile = spear_stem_profile_;
          active_profile_name = stem_has_head_support ? "spear_stem" : "spear_stem_no_head";
        } else {
          main_eval.profile = head_eval.profile;
          profile = spear_head_profile_;
          active_profile_name = "spear_head";
        }
        main_eval_raw_present = false;
      }
    } else {
      main_eval = evaluateProfile(
        tip_type, profile, preview_bgr, color_depth, distance_mask,
        detect_roi, background_depth);
      active_profile_name = tip_type;
      main_eval_raw_present = main_eval.rawPresent();
    }

    cv::Mat foreground_mask = main_eval.foreground_mask;
    cv::Mat dark_mask = main_eval.dark_mask;
    cv::Mat candidate_mask = main_eval.candidate_mask;

    const int distance_pixels = cv::countNonZero(distance_mask(detect_roi));
    const int foreground_pixels = main_eval.foreground_pixels;
    const int dark_pixels = main_eval.dark_pixels;
    const int candidate_pixels = main_eval.candidate_pixels;

    const int raw_component_count = main_eval.raw_component_count;
    const int accepted_candidate_count = main_eval.accepted_candidate_count;

    Candidate best = main_eval.best;

    const bool raw_present = main_eval_raw_present;
    const bool stable_present = updateStableResult(raw_present);

    cv::Mat overlay = preview_bgr.clone();

    if (display_mask_mode_ == "all") {
      cv::Mat dist_vis = cv::Mat::zeros(distance_mask.size(), CV_8UC1);
      cv::Mat fg_vis = cv::Mat::zeros(foreground_mask.size(), CV_8UC1);
      cv::Mat dark_vis = cv::Mat::zeros(dark_mask.size(), CV_8UC1);
      distance_mask(detect_roi).copyTo(dist_vis(detect_roi));
      foreground_mask(detect_roi).copyTo(fg_vis(detect_roi));
      dark_mask(detect_roi).copyTo(dark_vis(detect_roi));
      overlayMaskColor(overlay, dist_vis, cv::Scalar(255, 0, 0), detect_roi);      // blue
      overlayMaskColor(overlay, fg_vis, cv::Scalar(0, 255, 0), detect_roi);        // green
      overlayMaskColor(overlay, dark_vis, cv::Scalar(255, 0, 255), detect_roi);    // magenta
      cv::addWeighted(overlay, 0.55, preview_bgr, 0.45, 0.0, preview_bgr);
    } else {
      cv::Mat debug_mask = selectSingleDebugMask(
        distance_mask,
        foreground_mask,
        dark_mask,
        candidate_mask,
        detect_roi);
      overlay.setTo(cv::Scalar(0, 255, 0), debug_mask);
      cv::addWeighted(overlay, 0.35, preview_bgr, 0.65, 0.0, preview_bgr);
    }

    std_msgs::msg::UInt8 present_msg;
    present_msg.data = stable_present ? 1 : 0;
    current_present_pub_->publish(present_msg);

    const cv::Scalar box_color =
      stable_present ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

    cv::rectangle(preview_bgr, display_roi, box_color, 2);
    cv::rectangle(preview_bgr, detect_roi, cv::Scalar(255, 255, 0), 1);

    if (best.exists) {
      const cv::Scalar candidate_color =
        best.accepted ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 165, 255);
      cv::rectangle(preview_bgr, best.bbox, candidate_color, 2);
      cv::circle(preview_bgr, best.center, 4, candidate_color, -1);
    }

    int history_true_count = 0;
    for (const bool v : stable_history_) {
      if (v) {
        ++history_true_count;
      }
    }

    drawText(
      preview_bgr,
      "slot " + std::to_string(current_slot_id_) + " / expected: " + tip_type,
      30,
      box_color,
      0.70);

    drawText(
      preview_bgr,
      std::string("present: ") + (stable_present ? "YES" : "NO") +
        " raw: " + (raw_present ? "YES" : "NO") +
        " stable: " + (stable_enabled_ ? "ON" : "OFF"),
      58,
      box_color,
      0.60);

    drawText(
      preview_bgr,
      "mode:" + display_mask_mode_ +
        " prof:" + active_profile_name +
        " cand_mode:" + profile.candidate_mask_mode,
      84,
      cv::Scalar(0, 255, 255),
      0.46);

    drawText(
      preview_bgr,
      "dist:" + std::to_string(distance_pixels) +
        " fg:" + std::to_string(foreground_pixels) +
        " dark:" + std::to_string(dark_pixels) +
        " cand:" + std::to_string(candidate_pixels),
      108,
      cv::Scalar(0, 255, 255),
      0.46);

    drawText(
      preview_bgr,
      "bg:" + std::to_string(background_depth).substr(0, 5) +
        " valid:" + std::to_string(background_valid_count) +
        " comp:" + std::to_string(raw_component_count) +
        " pass:" + std::to_string(accepted_candidate_count),
      132,
      cv::Scalar(0, 255, 255),
      0.46);

    if (best.exists) {
      drawText(
        preview_bgr,
        "best:" + best.rejected_reason +
          " score:" + std::to_string(best.final_score).substr(0, 5) +
          "/" + std::to_string(profile.min_candidate_score).substr(0, 5) +
          " diff:" + std::to_string(best.mean_depth_diff).substr(0, 5) +
          " area:" + std::to_string(best.area),
        156,
        cv::Scalar(0, 255, 255),
        0.44);

      drawText(
        preview_bgr,
        "shape:" + std::to_string(best.shape_score).substr(0, 4) +
          " dep:" + std::to_string(best.depth_score).substr(0, 4) +
          " pos:" + std::to_string(best.position_score).substr(0, 4) +
          " dark:" + std::to_string(best.dark_score).substr(0, 4) +
          " dr:" + std::to_string(best.dark_ratio).substr(0, 4),
        180,
        cv::Scalar(255, 255, 0),
        0.42);

      drawText(
        preview_bgr,
        "bbox:" + std::to_string(best.bbox.x) + "," +
          std::to_string(best.bbox.y) + "," +
          std::to_string(best.bbox.width) + "," +
          std::to_string(best.bbox.height) +
          " cx:" + std::to_string(best.center_x_ratio).substr(0, 4) +
          " cy:" + std::to_string(best.center_y_ratio).substr(0, 4) +
          " dcnt:" + std::to_string(best.depth_count),
        204,
        cv::Scalar(255, 255, 0),
        0.40);
    } else {
      drawText(preview_bgr, "best:none", 156, cv::Scalar(0, 165, 255), 0.50);
    }

    drawText(
      preview_bgr,
      "history:" + std::to_string(history_true_count) + "/" +
        std::to_string(static_cast<int>(stable_history_.size())) +
        " need:" + std::to_string(std::clamp(stable_accept_count_, 1, std::max(1, stable_history_size_))),
      228,
      cv::Scalar(255, 255, 0),
      0.40);

    if (debug_show_roi_info_) {
      drawText(
        preview_bgr,
        "roi:" + std::to_string(display_roi.x) + "," +
          std::to_string(display_roi.y) + "," +
          std::to_string(display_roi.width) + "," +
          std::to_string(display_roi.height),
        250,
        cv::Scalar(255, 255, 0),
        0.38);
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "slot=%d expected=%s profile=%s present=%s raw=%s stable=%s mode=%s cand_mode=%s dist=%d fg=%d dark=%d cand=%d bg=%.3f bg_valid=%d comp=%d pass=%d best=%s accepted=%s reason=%s score=%.3f diff=%.3f area=%d bbox=(%d,%d,%d,%d) pos=(%.2f,%.2f) dark_ratio=%.2f depth_count=%d",
      current_slot_id_,
      tip_type.c_str(),
      active_profile_name.c_str(),
      stable_present ? "true" : "false",
      raw_present ? "true" : "false",
      stable_enabled_ ? "on" : "off",
      display_mask_mode_.c_str(),
      profile.candidate_mask_mode.c_str(),
      distance_pixels,
      foreground_pixels,
      dark_pixels,
      candidate_pixels,
      background_depth,
      background_valid_count,
      raw_component_count,
      accepted_candidate_count,
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

    showPreview(preview_bgr);
  }

  void showPreview(cv::Mat & preview_bgr)
  {
    if (!enable_cv_preview_) {
      return;
    }

    if (preview_scale_ > 0.0 && std::abs(preview_scale_ - 1.0) > 1e-6) {
      cv::resize(preview_bgr, preview_bgr, cv::Size(), preview_scale_, preview_scale_, cv::INTER_LINEAR);
    }

    cv::imshow("current_tip_detector_preview", preview_bgr);
    cv::waitKey(std::max(1, cv_wait_key_ms_));
  }

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

  double background_percentile_{0.75};
  int min_background_valid_pixels_{50};
  double foreground_max_depth_diff_{0.18};

  int morph_open_size_{0};
  int morph_close_size_{5};
  int mask_dilate_size_{5};

  bool stable_enabled_{false};
  int stable_history_size_{5};
  int stable_accept_count_{3};
  std::deque<bool> stable_history_;

  double detect_roi_height_ratio_{1.0};

  bool enable_cv_preview_{true};
  double preview_scale_{0.8};
  int cv_wait_key_ms_{1};

  std::string display_mask_mode_{"all"};
  bool debug_show_roi_info_{true};

  std::vector<double> slot_roi_ratios_;

  TipProfile spear_profile_;
  bool spear_enable_dual_profile_{true};
  bool spear_stem_require_head_support_{true};
  double spear_stem_head_support_above_ratio_{0.60};
  double spear_stem_head_support_expand_x_ratio_{0.28};
  int spear_stem_head_support_min_dark_pixels_{140};
  double spear_stem_head_support_min_dark_ratio_{0.06};
  TipProfile spear_head_profile_;
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
};

}  // namespace weapon_tip_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<weapon_tip_detector::CurrentTipDetectorNode>());
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}