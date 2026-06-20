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
  // depth_candidate, dark, depth_candidate_or_dark, depth_candidate_and_dark, distance, distance_or_dark.
  std::string candidate_mask_mode{"foreground"};

  double min_depth_delta{0.02};
  int min_component_area{20};
  double min_candidate_score{0.30};
  double max_component_area_ratio{0.60};
  bool require_depth_for_candidate{true};

  // Optional veto: depth is not required, but if enough valid depth points
  // prove the candidate is behind the estimated background, reject it.
  bool enable_depth_too_far_veto{false};
  int depth_too_far_veto_min_count{120};
  double depth_too_far_veto_max_delta{-0.003};

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
  double mean_depth_delta{0.0};

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

  cv::Mat depth_candidate_mask;
  cv::Mat dark_mask;
  cv::Mat candidate_mask;

  int depth_candidate_pixels{0};
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

}  // namespace weapon_tip_detector

#endif  // WEAPON_TIP_DETECTOR__DETECTOR_TYPES_HPP_