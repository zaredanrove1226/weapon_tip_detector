#ifndef WEAPON_TIP_DETECTOR__DETECTION_PIPELINE_HPP_
#define WEAPON_TIP_DETECTOR__DETECTION_PIPELINE_HPP_

#include "weapon_tip_detector/detector_types.hpp"

#include <opencv2/core.hpp>

#include <deque>
#include <string>

namespace weapon_tip_detector
{

struct SupportRuleConfig
{
  bool require_support{true};
  double above_ratio{0.55};
  double expand_x_ratio{0.22};
  int min_dark_pixels{120};
  double min_dark_ratio{0.06};
};

struct DetectionPipelineConfig
{
  double target_distance{0.30};
  double valid_depth_min{0.12};
  double valid_depth_max{0.70};

  double background_percentile{0.75};
  int min_background_valid_pixels{50};
  double foreground_max_depth_diff{0.18};

  int morph_open_size{0};
  int morph_close_size{5};
  int mask_dilate_size{5};

  bool stable_enabled{false};
  int stable_history_size{5};
  int stable_accept_count{3};

  bool enable_fist_dual_profile{true};
  bool spear_enable_dual_profile{true};

  SupportRuleConfig fist_stem_body_support;
  SupportRuleConfig spear_stem_head_support;
};

struct ProfileBundle
{
  TipProfile spear;
  TipProfile spear_head;
  TipProfile spear_stem;

  TipProfile fist;
  TipProfile fist_body;
  TipProfile fist_stem;

  TipProfile palm;
};

struct DetectionResult
{
  bool raw_present{false};
  bool stable_present{false};

  std::string tip_type{"unknown"};
  std::string active_profile_name{"unknown"};
  TipProfile active_profile;

  cv::Rect display_roi;
  cv::Rect detect_roi;

  double background_depth{0.0};
  int background_valid_count{0};

  int distance_pixels{0};
  cv::Mat distance_mask;

  int stable_true_count{0};
  int stable_history_count{0};

  ProfileEvaluation main_eval;
};

class DetectionPipeline
{
public:
  explicit DetectionPipeline(const DetectionPipelineConfig & config);

  void setConfig(const DetectionPipelineConfig & config);

  const DetectionPipelineConfig & config() const;

  void resetStableHistory();

  DetectionResult process(
    const cv::Mat & bgr_image,
    const cv::Mat & color_depth,
    const cv::Mat & distance_mask,
    const cv::Rect & display_roi,
    const cv::Rect & detect_roi,
    const std::string & tip_type,
    const TipProfile & main_profile,
    const ProfileBundle & profiles);

private:
  double estimateBackgroundDepth(
    const cv::Mat & color_depth,
    const cv::Rect & roi,
    int & valid_count) const;

  cv::Mat buildForegroundMask(
    const cv::Mat & color_depth,
    const cv::Rect & detect_roi,
    double background_depth,
    const TipProfile & profile) const;

  cv::Mat buildDarkMask(
    const cv::Mat & bgr_image,
    const cv::Rect & detect_roi,
    const TipProfile & profile) const;

  cv::Mat buildCandidateMask(
    const cv::Mat & distance_mask,
    const cv::Mat & foreground_mask,
    const cv::Mat & dark_mask,
    const cv::Rect & detect_roi,
    const TipProfile & profile) const;

  void applyIgnoreMask(
    cv::Mat & candidate_mask,
    const cv::Rect & detect_roi,
    const TipProfile & profile) const;

  bool isSuppressedByRoiEdge(
    const Candidate & candidate,
    const TipProfile & profile) const;

  Candidate scoreCandidate(
    int label,
    const cv::Mat & labels,
    const cv::Mat & stats,
    const cv::Mat & centroids,
    const cv::Mat & color_depth_roi,
    const cv::Mat & dark_mask_roi,
    const cv::Rect & local_detect_roi,
    double background_depth,
    const TipProfile & profile) const;

  Candidate findBestCandidate(
    const cv::Mat & candidate_mask,
    const cv::Mat & color_depth,
    const cv::Mat & dark_mask,
    const cv::Rect & detect_roi,
    double background_depth,
    const TipProfile & profile,
    int & raw_component_count,
    int & accepted_candidate_count) const;

  ProfileEvaluation evaluateProfile(
    const std::string & name,
    const TipProfile & profile,
    const cv::Mat & bgr_image,
    const cv::Mat & color_depth,
    const cv::Mat & distance_mask,
    const cv::Rect & detect_roi,
    double background_depth) const;

  static Candidate chooseBestCandidate(
    const Candidate & a,
    const Candidate & b);

  bool hasUpperDarkSupport(
    const Candidate & lower_candidate,
    const cv::Mat & upper_dark_mask,
    const cv::Rect & detect_roi,
    double above_ratio,
    double expand_x_ratio,
    int min_dark_pixels,
    double min_dark_ratio,
    int & support_dark_pixels,
    double & support_dark_ratio) const;

  bool hasSupportByRule(
    const Candidate & lower_candidate,
    const cv::Mat & upper_dark_mask,
    const cv::Rect & detect_roi,
    const SupportRuleConfig & rule,
    int & support_dark_pixels,
    double & support_dark_ratio) const;

  bool updateStableResult(bool raw_present);

  void updateStableDebugCounts(
    int & true_count,
    int & history_count) const;

  DetectionPipelineConfig config_;
  std::deque<bool> stable_history_;
};

}  // namespace weapon_tip_detector

#endif  // WEAPON_TIP_DETECTOR__DETECTION_PIPELINE_HPP_