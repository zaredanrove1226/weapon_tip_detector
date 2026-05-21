#include "weapon_tip_detector/detection_pipeline.hpp"

#include "weapon_tip_detector/detector_utils.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace weapon_tip_detector
{

DetectionPipeline::DetectionPipeline(const DetectionPipelineConfig & config)
: config_(config)
{
}

void DetectionPipeline::setConfig(const DetectionPipelineConfig & config)
{
  config_ = config;
}

const DetectionPipelineConfig & DetectionPipeline::config() const
{
  return config_;
}

void DetectionPipeline::resetStableHistory()
{
  stable_history_.clear();
}

DetectionResult DetectionPipeline::process(
  const cv::Mat & bgr_image,
  const cv::Mat & color_depth,
  const cv::Mat & distance_mask,
  const cv::Rect & display_roi,
  const cv::Rect & detect_roi,
  const std::string & tip_type,
  const TipProfile & main_profile,
  const ProfileBundle & profiles)
{
  DetectionResult result;
  result.tip_type = tip_type;
  result.active_profile_name = tip_type;
  result.active_profile = main_profile;
  result.display_roi = display_roi;
  result.detect_roi = detect_roi;

  if (detect_roi.width <= 0 || detect_roi.height <= 0) {
    result.raw_present = false;
    result.stable_present = updateStableResult(false);
    updateStableDebugCounts(result.stable_true_count, result.stable_history_count);
    return result;
  }

  if (!distance_mask.empty()) {
    result.distance_mask = distance_mask;
    result.distance_pixels = cv::countNonZero(distance_mask(detect_roi));
  }

  result.background_depth = estimateBackgroundDepth(
    color_depth,
    display_roi,
    result.background_valid_count);

  ProfileEvaluation main_eval;
  bool main_eval_raw_present = false;

  if (tip_type == "fist" && config_.enable_fist_dual_profile) {
    ProfileEvaluation body_eval = evaluateProfile(
      "fist_body",
      profiles.fist_body,
      bgr_image,
      color_depth,
      distance_mask,
      detect_roi,
      result.background_depth);

    ProfileEvaluation stem_eval = evaluateProfile(
      "fist_stem",
      profiles.fist_stem,
      bgr_image,
      color_depth,
      distance_mask,
      detect_roi,
      result.background_depth);

    int stem_support_dark_pixels = 0;
    double stem_support_dark_ratio = 0.0;

    const bool stem_has_body_support = hasSupportByRule(
      stem_eval.best,
      body_eval.dark_mask,
      detect_roi,
      config_.fist_stem_body_support,
      stem_support_dark_pixels,
      stem_support_dark_ratio);

    const bool body_ok = body_eval.best.exists && body_eval.best.accepted;
    bool stem_ok = stem_eval.best.exists && stem_eval.best.accepted;

    // fist_body 可以单独确认 fist；
    // fist_stem 不能单独确认 fist，必须有上方/附近拳体暗色支撑。
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
    main_eval.accepted_candidate_count =
      body_eval.accepted_candidate_count + stem_eval.accepted_candidate_count;

    if (body_ok) {
      main_eval.best = body_eval.best;
      main_eval.profile = body_eval.profile;
      result.active_profile = profiles.fist_body;
      result.active_profile_name = "fist_body";
      main_eval_raw_present = true;
    } else if (stem_ok) {
      main_eval.best = stem_eval.best;
      main_eval.profile = stem_eval.profile;
      result.active_profile = profiles.fist_stem;
      result.active_profile_name = "fist_stem";
      main_eval_raw_present = true;
    } else {
      main_eval.best = chooseBestCandidate(body_eval.best, stem_eval.best);

      if (main_eval.best.source_profile == "fist_stem") {
        main_eval.profile = stem_eval.profile;
        result.active_profile = profiles.fist_stem;
        result.active_profile_name = stem_has_body_support ? "fist_stem" : "fist_stem_no_body";
      } else {
        main_eval.profile = body_eval.profile;
        result.active_profile = profiles.fist_body;
        result.active_profile_name = "fist_body";
      }

      main_eval_raw_present = false;
    }
  } else if (tip_type == "spear" && config_.spear_enable_dual_profile) {
    ProfileEvaluation head_eval = evaluateProfile(
      "spear_head",
      profiles.spear_head,
      bgr_image,
      color_depth,
      distance_mask,
      detect_roi,
      result.background_depth);

    ProfileEvaluation stem_eval = evaluateProfile(
      "spear_stem",
      profiles.spear_stem,
      bgr_image,
      color_depth,
      distance_mask,
      detect_roi,
      result.background_depth);

    int stem_support_dark_pixels = 0;
    double stem_support_dark_ratio = 0.0;

    const bool stem_has_head_support = hasSupportByRule(
      stem_eval.best,
      head_eval.dark_mask,
      detect_roi,
      config_.spear_stem_head_support,
      stem_support_dark_pixels,
      stem_support_dark_ratio);

    const bool head_ok = head_eval.best.exists && head_eval.best.accepted;
    bool stem_ok = stem_eval.best.exists && stem_eval.best.accepted;

    // spear_head 可以单独确认 spear；
    // spear_stem 不能单独确认 spear，必须有上方/附近矛尖主体暗色支撑。
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
    main_eval.accepted_candidate_count =
      head_eval.accepted_candidate_count + stem_eval.accepted_candidate_count;

    if (head_ok) {
      main_eval.best = head_eval.best;
      main_eval.profile = head_eval.profile;
      result.active_profile = profiles.spear_head;
      result.active_profile_name = "spear_head";
      main_eval_raw_present = true;
    } else if (stem_ok) {
      main_eval.best = stem_eval.best;
      main_eval.profile = stem_eval.profile;
      result.active_profile = profiles.spear_stem;
      result.active_profile_name = "spear_stem";
      main_eval_raw_present = true;
    } else {
      main_eval.best = chooseBestCandidate(head_eval.best, stem_eval.best);

      if (main_eval.best.source_profile == "spear_stem") {
        main_eval.profile = stem_eval.profile;
        result.active_profile = profiles.spear_stem;
        result.active_profile_name = stem_has_head_support ? "spear_stem" : "spear_stem_no_head";
      } else {
        main_eval.profile = head_eval.profile;
        result.active_profile = profiles.spear_head;
        result.active_profile_name = "spear_head";
      }

      main_eval_raw_present = false;
    }
  } else if (tip_type == "palm") {
    main_eval = evaluatePalmByDensity(
      "palm_density",
      main_profile,
      bgr_image,
      color_depth,
      distance_mask,
      detect_roi,
      result.background_depth);

    result.active_profile = main_profile;
    result.active_profile_name = "palm_density";
    main_eval_raw_present = main_eval.rawPresent();
  } else {
    main_eval = evaluateProfile(
      tip_type,
      main_profile,
      bgr_image,
      color_depth,
      distance_mask,
      detect_roi,
      result.background_depth);

    result.active_profile = main_profile;
    result.active_profile_name = tip_type;
    main_eval_raw_present = main_eval.rawPresent();
  }

  result.main_eval = main_eval;
  result.raw_present = main_eval_raw_present;
  result.stable_present = updateStableResult(result.raw_present);
  updateStableDebugCounts(result.stable_true_count, result.stable_history_count);

  return result;
}

double DetectionPipeline::estimateBackgroundDepth(
  const cv::Mat & color_depth,
  const cv::Rect & roi,
  int & valid_count) const
{
  std::vector<float> depths;
  depths.reserve(static_cast<size_t>(std::max(0, roi.area())));

  valid_count = 0;

  if (color_depth.empty() || roi.width <= 0 || roi.height <= 0) {
    return config_.target_distance;
  }

  for (int y = roi.y; y < roi.y + roi.height; ++y) {
    for (int x = roi.x; x < roi.x + roi.width; ++x) {
      const float z = color_depth.at<float>(y, x);

      if (!std::isfinite(z)) {
        continue;
      }

      if (z < config_.valid_depth_min || z > config_.valid_depth_max) {
        continue;
      }

      depths.push_back(z);
    }
  }

  valid_count = static_cast<int>(depths.size());

  if (valid_count < config_.min_background_valid_pixels) {
    return config_.target_distance;
  }

  const double bg = percentile(depths, config_.background_percentile);

  if (!std::isfinite(bg)) {
    return config_.target_distance;
  }

  return bg;
}

cv::Mat DetectionPipeline::buildForegroundMask(
  const cv::Mat & color_depth,
  const cv::Rect & detect_roi,
  const double background_depth,
  const TipProfile & profile) const
{
  cv::Mat fg_mask = cv::Mat::zeros(color_depth.size(), CV_8UC1);

  if (color_depth.empty() || detect_roi.width <= 0 || detect_roi.height <= 0) {
    return fg_mask;
  }

  for (int y = detect_roi.y; y < detect_roi.y + detect_roi.height; ++y) {
    for (int x = detect_roi.x; x < detect_roi.x + detect_roi.width; ++x) {
      const float z_f = color_depth.at<float>(y, x);

      if (!std::isfinite(z_f)) {
        continue;
      }

      const double z = static_cast<double>(z_f);

      if (z < config_.valid_depth_min || z > config_.valid_depth_max) {
        continue;
      }

      const double diff = background_depth - z;

      if (diff >= profile.foreground_min_depth_diff &&
          diff <= config_.foreground_max_depth_diff)
      {
        fg_mask.at<uint8_t>(y, x) = 255;
      }
    }
  }

  if (cv::countNonZero(fg_mask) == 0) {
    return fg_mask;
  }

  if (config_.mask_dilate_size > 1) {
    const int k = config_.mask_dilate_size;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    cv::dilate(fg_mask, fg_mask, kernel);
  }

  if (config_.morph_close_size > 1) {
    const int k = config_.morph_close_size;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_CLOSE, kernel);
  }

  if (config_.morph_open_size > 1) {
    const int k = config_.morph_open_size;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_OPEN, kernel);
  }

  return fg_mask;
}

cv::Mat DetectionPipeline::buildDarkMask(
  const cv::Mat & bgr_image,
  const cv::Rect & detect_roi,
  const TipProfile & profile) const
{
  cv::Mat dark_mask = cv::Mat::zeros(bgr_image.size(), CV_8UC1);

  if (bgr_image.empty() || detect_roi.width <= 0 || detect_roi.height <= 0) {
    return dark_mask;
  }

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

cv::Mat DetectionPipeline::buildCandidateMask(
  const cv::Mat & distance_mask,
  const cv::Mat & foreground_mask,
  const cv::Mat & dark_mask,
  const cv::Rect & detect_roi,
  const TipProfile & profile) const
{
  cv::Mat candidate_mask = cv::Mat::zeros(foreground_mask.size(), CV_8UC1);

  if (detect_roi.width <= 0 || detect_roi.height <= 0) {
    return candidate_mask;
  }

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

void DetectionPipeline::applyIgnoreMask(
  cv::Mat & candidate_mask,
  const cv::Rect & detect_roi,
  const TipProfile & profile) const
{
  if (!profile.enable_ignore_mask || profile.ignore_rects.empty()) {
    return;
  }

  if (candidate_mask.empty() || detect_roi.width <= 0 || detect_roi.height <= 0) {
    return;
  }

  for (const auto & r : profile.ignore_rects) {
    const double x_ratio = std::clamp(r.x, 0.0, 1.0);
    const double y_ratio = std::clamp(r.y, 0.0, 1.0);
    const double w_ratio = std::clamp(r.width, 0.0, 1.0);
    const double h_ratio = std::clamp(r.height, 0.0, 1.0);

    const int x = detect_roi.x + static_cast<int>(
      std::lround(x_ratio * static_cast<double>(detect_roi.width)));
    const int y = detect_roi.y + static_cast<int>(
      std::lround(y_ratio * static_cast<double>(detect_roi.height)));
    const int w = static_cast<int>(
      std::lround(w_ratio * static_cast<double>(detect_roi.width)));
    const int h = static_cast<int>(
      std::lround(h_ratio * static_cast<double>(detect_roi.height)));

    cv::Rect ignore_rect(x, y, std::max(0, w), std::max(0, h));
    ignore_rect &= detect_roi;

    if (ignore_rect.width <= 0 || ignore_rect.height <= 0) {
      continue;
    }

    candidate_mask(ignore_rect).setTo(0);
  }
}

bool DetectionPipeline::isSuppressedByRoiEdge(
  const Candidate & c,
  const TipProfile & profile) const
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

Candidate DetectionPipeline::scoreCandidate(
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
  c.width_ratio = static_cast<double>(w) /
    static_cast<double>(std::max(1, local_detect_roi.width));
  c.height_ratio = static_cast<double>(h) /
    static_cast<double>(std::max(1, local_detect_roi.height));
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
    c.mask_count > 0 ?
    static_cast<double>(c.dark_count) / static_cast<double>(c.mask_count) :
    0.0;

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
    (profile.require_depth_for_candidate || c.depth_count > 0 ?
      std::max(0.0, profile.depth_score_weight) : 0.0) +
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

Candidate DetectionPipeline::findBestCandidate(
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

ProfileEvaluation DetectionPipeline::evaluateProfile(
  const std::string & name,
  const TipProfile & profile,
  const cv::Mat & bgr_image,
  const cv::Mat & color_depth,
  const cv::Mat & distance_mask,
  const cv::Rect & detect_roi,
  const double background_depth) const
{
  ProfileEvaluation ev;
  ev.name = name;
  ev.profile = profile;

  ev.foreground_mask = buildForegroundMask(color_depth, detect_roi, background_depth, profile);
  ev.dark_mask = buildDarkMask(bgr_image, detect_roi, profile);
  ev.candidate_mask = buildCandidateMask(
    distance_mask,
    ev.foreground_mask,
    ev.dark_mask,
    detect_roi,
    profile);

  applyIgnoreMask(ev.candidate_mask, detect_roi, profile);

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

cv::Rect DetectionPipeline::makeRelativeRect(
  const cv::Rect & roi,
  const double x_ratio,
  const double y_ratio,
  const double w_ratio,
  const double h_ratio) const
{
  const double xr = std::clamp(x_ratio, 0.0, 1.0);
  const double yr = std::clamp(y_ratio, 0.0, 1.0);
  const double wr = std::clamp(w_ratio, 0.0, 1.0);
  const double hr = std::clamp(h_ratio, 0.0, 1.0);

  cv::Rect r(
    roi.x + static_cast<int>(std::lround(xr * static_cast<double>(roi.width))),
    roi.y + static_cast<int>(std::lround(yr * static_cast<double>(roi.height))),
    static_cast<int>(std::lround(wr * static_cast<double>(roi.width))),
    static_cast<int>(std::lround(hr * static_cast<double>(roi.height))));

  r &= roi;
  return r;
}

double DetectionPipeline::maskDensity(
  const cv::Mat & mask,
  const cv::Rect & roi,
  int & pixels) const
{
  pixels = 0;

  if (mask.empty() || roi.width <= 0 || roi.height <= 0) {
    return 0.0;
  }

  pixels = cv::countNonZero(mask(roi));

  return static_cast<double>(pixels) /
    static_cast<double>(std::max(1, roi.area()));
}

ProfileEvaluation DetectionPipeline::evaluatePalmByDensity(
  const std::string & name,
  const TipProfile & profile,
  const cv::Mat & bgr_image,
  const cv::Mat & color_depth,
  const cv::Mat & distance_mask,
  const cv::Rect & detect_roi,
  const double background_depth) const
{
  (void)distance_mask;

  ProfileEvaluation ev;
  ev.name = name;
  ev.profile = profile;

  ev.foreground_mask = buildForegroundMask(color_depth, detect_roi, background_depth, profile);
  ev.dark_mask = buildDarkMask(bgr_image, detect_roi, profile);
  ev.candidate_mask = cv::Mat::zeros(bgr_image.size(), CV_8UC1);

  if (detect_roi.width <= 0 || detect_roi.height <= 0 || ev.dark_mask.empty()) {
    ev.best.exists = false;
    ev.best.accepted = false;
    ev.best.rejected_reason = "invalid_roi";
    return ev;
  }

  // 这三个区域对应你用天蓝色标出的人眼可见 palm 区域。
  // 注意：坐标是 detect_roi 内归一化坐标，不是整张图坐标。
  const cv::Rect body_left = makeRelativeRect(detect_roi, 0.18, 0.48, 0.24, 0.18);
  const cv::Rect body_mid = makeRelativeRect(detect_roi, 0.35, 0.45, 0.27, 0.21);
  const cv::Rect lower_stem = makeRelativeRect(detect_roi, 0.34, 0.62, 0.18, 0.28);

  int left_pixels = 0;
  int mid_pixels = 0;
  int stem_pixels = 0;

  const double left_density = maskDensity(ev.dark_mask, body_left, left_pixels);
  const double mid_density = maskDensity(ev.dark_mask, body_mid, mid_pixels);
  const double stem_density = maskDensity(ev.dark_mask, lower_stem, stem_pixels);

  int votes = 0;

  if (left_pixels >= 20 && left_density >= 0.08) {
    ++votes;
    ev.dark_mask(body_left).copyTo(ev.candidate_mask(body_left));
  }

  if (mid_pixels >= 20 && mid_density >= 0.08) {
    ++votes;
    ev.dark_mask(body_mid).copyTo(ev.candidate_mask(body_mid));
  }

  if (stem_pixels >= 15 && stem_density >= 0.06) {
    ++votes;
    ev.dark_mask(lower_stem).copyTo(ev.candidate_mask(lower_stem));
  }

  const int total_pixels = left_pixels + mid_pixels + stem_pixels;
  const bool palm_present = votes >= 2 && total_pixels >= 50;

  ev.foreground_pixels = cv::countNonZero(ev.foreground_mask(detect_roi));
  ev.dark_pixels = cv::countNonZero(ev.dark_mask(detect_roi));
  ev.candidate_pixels = cv::countNonZero(ev.candidate_mask(detect_roi));

  ev.raw_component_count = votes;
  ev.accepted_candidate_count = palm_present ? 1 : 0;

  Candidate c;
  c.exists = total_pixels > 0;
  c.accepted = palm_present;
  c.source_profile = name;
  c.rejected_reason = palm_present ? "density_votes" : "density_low";

  c.area = total_pixels;
  c.mask_count = total_pixels;
  c.dark_count = total_pixels;
  c.dark_ratio = total_pixels > 0 ? 1.0 : 0.0;
  c.depth_count = 0;
  c.mean_depth = 0.0;
  c.mean_depth_diff = 0.0;

  c.final_score = static_cast<double>(votes) / 3.0;
  c.shape_score = c.final_score;
  c.area_score = std::min(1.0, static_cast<double>(total_pixels) / 120.0);
  c.position_score = 1.0;
  c.dark_score = c.dark_ratio;
  c.depth_score = 0.0;

  c.bbox = body_left | body_mid | lower_stem;
  c.center = cv::Point2d(
    static_cast<double>(c.bbox.x) + static_cast<double>(c.bbox.width) * 0.5,
    static_cast<double>(c.bbox.y) + static_cast<double>(c.bbox.height) * 0.5);

  c.center_x_ratio =
    (c.center.x - static_cast<double>(detect_roi.x)) /
    static_cast<double>(std::max(1, detect_roi.width));

  c.center_y_ratio =
    (c.center.y - static_cast<double>(detect_roi.y)) /
    static_cast<double>(std::max(1, detect_roi.height));

  c.width_ratio =
    static_cast<double>(c.bbox.width) /
    static_cast<double>(std::max(1, detect_roi.width));

  c.height_ratio =
    static_cast<double>(c.bbox.height) /
    static_cast<double>(std::max(1, detect_roi.height));

  c.area_ratio =
    static_cast<double>(total_pixels) /
    static_cast<double>(std::max(1, detect_roi.area()));

  c.aspect_w_over_h =
    static_cast<double>(c.bbox.width) /
    static_cast<double>(std::max(1, c.bbox.height));

  c.fill_ratio =
    static_cast<double>(total_pixels) /
    static_cast<double>(std::max(1, c.bbox.area()));

  ev.best = c;
  return ev;
}

Candidate DetectionPipeline::chooseBestCandidate(
  const Candidate & a,
  const Candidate & b)
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

bool DetectionPipeline::hasUpperDarkSupport(
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

bool DetectionPipeline::hasSupportByRule(
  const Candidate & lower_candidate,
  const cv::Mat & upper_dark_mask,
  const cv::Rect & detect_roi,
  const SupportRuleConfig & rule,
  int & support_dark_pixels,
  double & support_dark_ratio) const
{
  if (!rule.require_support) {
    support_dark_pixels = 0;
    support_dark_ratio = 0.0;
    return true;
  }

  return hasUpperDarkSupport(
    lower_candidate,
    upper_dark_mask,
    detect_roi,
    rule.above_ratio,
    rule.expand_x_ratio,
    rule.min_dark_pixels,
    rule.min_dark_ratio,
    support_dark_pixels,
    support_dark_ratio);
}

bool DetectionPipeline::updateStableResult(const bool raw_present)
{
  if (!config_.stable_enabled) {
    stable_history_.clear();
    stable_history_.push_back(raw_present);
    return raw_present;
  }

  const int history_size = std::max(1, config_.stable_history_size);
  const int accept_count = std::clamp(config_.stable_accept_count, 1, history_size);

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

void DetectionPipeline::updateStableDebugCounts(
  int & true_count,
  int & history_count) const
{
  true_count = 0;

  for (const bool v : stable_history_) {
    if (v) {
      ++true_count;
    }
  }

  history_count = static_cast<int>(stable_history_.size());
}

}  // namespace weapon_tip_detector