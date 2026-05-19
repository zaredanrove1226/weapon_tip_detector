#include "weapon_tip_detector/preview_debugger.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>

namespace weapon_tip_detector
{

PreviewDebugger::PreviewDebugger(const PreviewDebuggerConfig & config)
: config_(config)
{
}

void PreviewDebugger::setConfig(const PreviewDebuggerConfig & config)
{
  config_ = config;
}

const PreviewDebuggerConfig & PreviewDebugger::config() const
{
  return config_;
}

void PreviewDebugger::createWindowIfEnabled() const
{
  if (config_.enable_cv_preview) {
    cv::namedWindow("current_tip_detector_preview", cv::WINDOW_NORMAL);
  }
}

void PreviewDebugger::drawInvalidRoi(cv::Mat & preview_bgr) const
{
  drawText(
    preview_bgr,
    "Invalid current_slot_id or ROI",
    40,
    cv::Scalar(0, 0, 255),
    0.8);
}

void PreviewDebugger::draw(
  cv::Mat & preview_bgr,
  const DetectionResult & result,
  const int current_slot_id) const
{
  if (!config_.enable_cv_preview || preview_bgr.empty()) {
    return;
  }

  const cv::Rect display_roi = result.display_roi;
  const cv::Rect detect_roi = result.detect_roi;
  const ProfileEvaluation & ev = result.main_eval;
  const Candidate & best = ev.best;

  if (display_roi.width <= 0 || display_roi.height <= 0 ||
      detect_roi.width <= 0 || detect_roi.height <= 0)
  {
    drawInvalidRoi(preview_bgr);
    return;
  }

  cv::Mat overlay = preview_bgr.clone();

  if (config_.display_mask_mode == "all") {
    if (!result.main_eval.foreground_mask.empty()) {
      overlayMaskColor(
        overlay,
        result.main_eval.foreground_mask,
        cv::Scalar(0, 255, 255),
        detect_roi);
    }

    if (!result.main_eval.dark_mask.empty()) {
      overlayMaskColor(
        overlay,
        result.main_eval.dark_mask,
        cv::Scalar(255, 0, 255),
        detect_roi);
    }

    if (!result.main_eval.candidate_mask.empty()) {
      overlayMaskColor(
        overlay,
        result.main_eval.candidate_mask,
        cv::Scalar(0, 255, 0),
        detect_roi);
    }
  } else {
    const cv::Mat debug_mask = selectSingleDebugMask(result);
    if (!debug_mask.empty()) {
      overlayMaskColor(
        overlay,
        debug_mask,
        cv::Scalar(0, 255, 0),
        detect_roi);
    }
  }

  overlay.copyTo(preview_bgr);

  cv::rectangle(preview_bgr, display_roi, cv::Scalar(255, 0, 0), 2);
  cv::rectangle(preview_bgr, detect_roi, cv::Scalar(0, 255, 255), 2);

  if (best.exists && best.bbox.width > 0 && best.bbox.height > 0) {
    const cv::Scalar bbox_color = best.accepted ?
      cv::Scalar(0, 255, 0) :
      cv::Scalar(0, 0, 255);

    cv::rectangle(preview_bgr, best.bbox, bbox_color, 2);

    cv::circle(
      preview_bgr,
      cv::Point(
        static_cast<int>(std::lround(best.center.x)),
        static_cast<int>(std::lround(best.center.y))),
      4,
      bbox_color,
      -1);
  }

  const char * present_text = result.stable_present ? "YES" : "NO";
  const char * raw_text = result.raw_present ? "YES" : "NO";
  const char * stable_text = result.stable_present ? "ON" : "OFF";

  cv::Scalar present_color = result.stable_present ?
    cv::Scalar(0, 255, 0) :
    cv::Scalar(0, 0, 255);

  std::ostringstream line1;
  line1 << "slot " << current_slot_id
        << " expected " << result.tip_type
        << " present " << present_text
        << " raw " << raw_text
        << " stable " << stable_text;

  drawText(preview_bgr, line1.str(), 32, present_color, 0.65, 2);

  std::ostringstream line2;
  line2 << "prof " << result.active_profile_name
        << " cond_mode " << result.active_profile.candidate_mask_mode
        << " mode " << config_.display_mask_mode;

  drawText(preview_bgr, line2.str(), 58, cv::Scalar(255, 255, 255), 0.50, 2);

  std::ostringstream line3;
  line3 << "dist " << result.distance_pixels
        << " fg " << ev.foreground_pixels
        << " dark " << ev.dark_pixels
        << " cond " << ev.candidate_pixels
        << " bg " << std::fixed << std::setprecision(3) << result.background_depth
        << " bg_valid " << result.background_valid_count;

  drawText(preview_bgr, line3.str(), 82, cv::Scalar(255, 255, 255), 0.48, 2);

  std::ostringstream line4;
  line4 << "comp " << ev.raw_component_count
        << " pass " << ev.accepted_candidate_count
        << " hist " << result.stable_true_count << "/" << result.stable_history_count;

  drawText(preview_bgr, line4.str(), 106, cv::Scalar(255, 255, 255), 0.48, 2);

  std::ostringstream line5;
  line5 << "best " << (best.exists ? "true" : "false")
        << " accepted " << (best.accepted ? "true" : "false")
        << " reason " << best.rejected_reason
        << " score " << std::fixed << std::setprecision(3)
        << best.final_score << "/" << result.active_profile.min_candidate_score;

  drawText(preview_bgr, line5.str(), 130, cv::Scalar(255, 255, 255), 0.48, 2);

  std::ostringstream line6;
  line6 << "diff " << std::fixed << std::setprecision(3) << best.mean_depth_diff
        << " area " << best.area
        << " bbox (" << best.bbox.x << "," << best.bbox.y << ","
        << best.bbox.width << "," << best.bbox.height << ")";

  drawText(preview_bgr, line6.str(), 154, cv::Scalar(255, 255, 255), 0.48, 2);

  std::ostringstream line7;
  line7 << "pos (" << std::fixed << std::setprecision(2)
        << best.center_x_ratio << "," << best.center_y_ratio << ")"
        << " dark_ratio " << std::setprecision(2) << best.dark_ratio
        << " depth_count " << best.depth_count;

  drawText(preview_bgr, line7.str(), 178, cv::Scalar(255, 255, 255), 0.48, 2);

  if (config_.debug_show_roi_info) {
    std::ostringstream roi_line;
    roi_line << "display_roi=("
             << display_roi.x << "," << display_roi.y << ","
             << display_roi.width << "," << display_roi.height << ") "
             << "detect_roi=("
             << detect_roi.x << "," << detect_roi.y << ","
             << detect_roi.width << "," << detect_roi.height << ")";

    drawText(
      preview_bgr,
      roi_line.str(),
      std::max(205, preview_bgr.rows - 18),
      cv::Scalar(255, 255, 255),
      0.45,
      1);
  }
}

void PreviewDebugger::show(cv::Mat & preview_bgr) const
{
  if (!config_.enable_cv_preview || preview_bgr.empty()) {
    return;
  }

  cv::Mat show_image = preview_bgr;

  if (config_.preview_scale > 0.05 &&
      std::abs(config_.preview_scale - 1.0) > 1e-6)
  {
    cv::resize(
      preview_bgr,
      show_image,
      cv::Size(),
      config_.preview_scale,
      config_.preview_scale,
      cv::INTER_AREA);
  }

  cv::imshow("current_tip_detector_preview", show_image);
  cv::waitKey(std::max(1, config_.cv_wait_key_ms));
}

void PreviewDebugger::drawText(
  cv::Mat & image,
  const std::string & text,
  const int y,
  const cv::Scalar & color,
  const double scale,
  const int thickness) const
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

cv::Mat PreviewDebugger::selectSingleDebugMask(
  const DetectionResult & result) const
{
  const cv::Rect detect_roi = result.detect_roi;

  if (result.main_eval.candidate_mask.empty()) {
    return cv::Mat();
  }

  cv::Mat debug_mask = cv::Mat::zeros(result.main_eval.candidate_mask.size(), CV_8UC1);

  if (detect_roi.width <= 0 || detect_roi.height <= 0) {
    return debug_mask;
  }

  if (config_.display_mask_mode == "distance") {
    if (!result.distance_mask.empty()) {
      result.distance_mask(detect_roi).copyTo(debug_mask(detect_roi));
    }
    return debug_mask;
  }

  if (config_.display_mask_mode == "foreground") {
    if (!result.main_eval.foreground_mask.empty()) {
      result.main_eval.foreground_mask(detect_roi).copyTo(debug_mask(detect_roi));
    }
    return debug_mask;
  }

  if (config_.display_mask_mode == "dark") {
    if (!result.main_eval.dark_mask.empty()) {
      result.main_eval.dark_mask(detect_roi).copyTo(debug_mask(detect_roi));
    }
    return debug_mask;
  }

  if (config_.display_mask_mode == "candidate") {
    if (!result.main_eval.candidate_mask.empty()) {
      result.main_eval.candidate_mask(detect_roi).copyTo(debug_mask(detect_roi));
    }
    return debug_mask;
  }

  cv::Mat combined = cv::Mat::zeros(result.main_eval.candidate_mask.size(), CV_8UC1);

  if (!result.main_eval.foreground_mask.empty()) {
    cv::bitwise_or(combined, result.main_eval.foreground_mask, combined);
  }

  if (!result.main_eval.dark_mask.empty()) {
    cv::bitwise_or(combined, result.main_eval.dark_mask, combined);
  }

  if (!result.main_eval.candidate_mask.empty()) {
    cv::bitwise_or(combined, result.main_eval.candidate_mask, combined);
  }

  combined(detect_roi).copyTo(debug_mask(detect_roi));
  return debug_mask;
}

void PreviewDebugger::overlayMaskColor(
  cv::Mat & overlay,
  const cv::Mat & mask,
  const cv::Scalar & color,
  const cv::Rect & roi) const
{
  if (mask.empty() || roi.width <= 0 || roi.height <= 0) {
    return;
  }

  if (cv::countNonZero(mask(roi)) <= 0) {
    return;
  }

  cv::Mat colored = cv::Mat::zeros(overlay.size(), overlay.type());
  colored.setTo(color, mask);
  cv::addWeighted(colored, 0.35, overlay, 1.0, 0.0, overlay);
}

}  // namespace weapon_tip_detector