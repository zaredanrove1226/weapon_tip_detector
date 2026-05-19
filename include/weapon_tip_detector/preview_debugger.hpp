#ifndef WEAPON_TIP_DETECTOR__PREVIEW_DEBUGGER_HPP_
#define WEAPON_TIP_DETECTOR__PREVIEW_DEBUGGER_HPP_

#include "weapon_tip_detector/detector_types.hpp"
#include "weapon_tip_detector/detection_pipeline.hpp"

#include <opencv2/core.hpp>

#include <string>

namespace weapon_tip_detector
{

struct PreviewDebuggerConfig
{
  bool enable_cv_preview{true};
  double preview_scale{0.8};
  int cv_wait_key_ms{1};

  // display_mask_mode: distance, foreground, dark, candidate, combined, all.
  std::string display_mask_mode{"all"};

  bool debug_show_roi_info{true};
};

class PreviewDebugger
{
public:
  explicit PreviewDebugger(const PreviewDebuggerConfig & config);

  void setConfig(const PreviewDebuggerConfig & config);

  const PreviewDebuggerConfig & config() const;

  void createWindowIfEnabled() const;

  void draw(
    cv::Mat & preview_bgr,
    const DetectionResult & result,
    int current_slot_id) const;

  void drawInvalidRoi(cv::Mat & preview_bgr) const;

  void show(cv::Mat & preview_bgr) const;

private:
  void drawText(
    cv::Mat & image,
    const std::string & text,
    int y,
    const cv::Scalar & color,
    double scale = 0.50,
    int thickness = 2) const;

  cv::Mat selectSingleDebugMask(
    const DetectionResult & result) const;

  void overlayMaskColor(
    cv::Mat & overlay,
    const cv::Mat & mask,
    const cv::Scalar & color,
    const cv::Rect & roi) const;

  PreviewDebuggerConfig config_;
};

}  // namespace weapon_tip_detector

#endif  // WEAPON_TIP_DETECTOR__PREVIEW_DEBUGGER_HPP_