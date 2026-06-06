#ifndef WEAPON_TIP_DETECTOR__DETECTOR_TYPES_HPP_
#define WEAPON_TIP_DETECTOR__DETECTOR_TYPES_HPP_

#include <opencv2/core.hpp>

#include <string>
#include <vector>

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

  // Optional veto: depth is not required, but if enough valid depth points
  // prove the candidate is behind the estimated background, reject it.
  bool enable_depth_behind_veto{false};
  int depth_behind_veto_min_count{120};
  double depth_behind_veto_max_diff{-0.003};

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

  bool enable_ignore_mask{false};
  std::vector<cv::Rect2d> ignore_rects;

  // Palm core-first detector.
  // The core ROI is the high-confidence palm body area.
  // The expanded ROI is derived from the core ROI by asymmetric expansion
  // and is only used to compensate small placement errors.
  bool enable_palm_body_core_check{true};
  cv::Rect2d palm_body_core_rect{0.28, 0.46, 0.42, 0.20};

  // Strong pass inside the core ROI.
  int palm_body_core_min_pixels{180};
  double palm_body_core_min_density{0.18};
  double palm_body_core_min_dark_ratio{0.80};

  // Weak evidence inside the core ROI.
  // Only weak-but-plausible evidence is allowed to trigger expanded search.
  // A completely empty core should normally stay rejected.
  int palm_body_core_weak_min_pixels{60};
  double palm_body_core_weak_min_density{0.06};
  double palm_body_core_weak_min_dark_ratio{0.35};

  // Asymmetric expansion ratios based on palm_body_core_rect.
  // Left/right are intentionally more tolerant than top/bottom by default.
  double palm_body_expand_left_ratio{0.35};
  double palm_body_expand_right_ratio{0.35};
  double palm_body_expand_top_ratio{0.20};
  double palm_body_expand_bottom_ratio{0.12};

  // Normal expanded search pass.
  int palm_body_expanded_min_pixels{150};
  double palm_body_expanded_min_density{0.14};
  double palm_body_expanded_min_dark_ratio{0.65};
  double palm_body_expanded_max_area_ratio{0.45};
  double palm_body_expanded_min_width_ratio{0.20};
  double palm_body_expanded_min_height_ratio{0.05};
  double palm_body_expanded_min_fill_ratio{0.10};
  double palm_body_expanded_min_aspect_w_over_h{1.20};
  double palm_body_expanded_max_aspect_w_over_h{8.00};
  double palm_body_expanded_max_center_shift_x_ratio{0.60};
  double palm_body_expanded_max_center_shift_y_ratio{0.45};
  double palm_body_expanded_edge_margin_ratio{0.03};

  // Very strong expanded pass.
  // This is only allowed when the core ROI is empty, so the thresholds are stricter.
  bool palm_body_enable_very_strong_empty_fallback{true};
  int palm_body_very_strong_min_pixels{220};
  double palm_body_very_strong_min_density{0.22};
  double palm_body_very_strong_min_dark_ratio{0.78};
  double palm_body_very_strong_max_area_ratio{0.38};
  double palm_body_very_strong_min_width_ratio{0.24};
  double palm_body_very_strong_min_height_ratio{0.06};
  double palm_body_very_strong_min_fill_ratio{0.14};
  double palm_body_very_strong_min_aspect_w_over_h{1.40};
  double palm_body_very_strong_max_aspect_w_over_h{7.00};
  double palm_body_very_strong_max_center_shift_x_ratio{0.45};
  double palm_body_very_strong_max_center_shift_y_ratio{0.32};

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

  // Palm core-first detector debug state.
  // Examples:
  // palm_core_pass
  // palm_core_weak
  // palm_core_empty
  // palm_expanded_pass
  // palm_expanded_reject
  // palm_expanded_very_strong_pass
  std::string palm_state{"none"};
  std::string palm_core_reason{"none"};
  std::string palm_expanded_reason{"none"};

  cv::Rect palm_core_roi;
  cv::Rect palm_expanded_roi;

  int palm_core_pixels{0};
  int palm_core_dark_pixels{0};
  double palm_core_density{0.0};
  double palm_core_dark_ratio{0.0};

  int palm_expanded_pixels{0};
  int palm_expanded_dark_pixels{0};
  double palm_expanded_density{0.0};
  double palm_expanded_dark_ratio{0.0};

  bool rawPresent() const
  {
    return best.exists && best.accepted;
  }
};

}  // namespace weapon_tip_detector

#endif  // WEAPON_TIP_DETECTOR__DETECTOR_TYPES_HPP_