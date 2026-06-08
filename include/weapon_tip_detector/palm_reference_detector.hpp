#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <map>

namespace weapon_tip_detector {

struct PalmReferenceSlotParams {
    std::string empty_rgb_path;
    std::string empty_depth_path;
    int dark_gray_threshold = 90;
    int rgb_diff_threshold = 35;
    double depth_delta = 0.04;
    bool allow_depth_hole = true;

    int min_area = 300;
    int min_width = 40;
    int min_height = 10;
    double min_aspect_ratio = 1.5;
    double min_fill_ratio = 0.18;

    int stable_required_frames = 3;
};

struct PalmReferenceResult {
    bool raw_present = false;
    cv::Mat candidate_mask;
    cv::Mat rgb_diff_mask;
    cv::Mat depth_diff_mask;
    cv::Rect best_bbox;
    int best_area = 0;
    int raw_component_count = 0;
    double final_score = 0.0;
    std::string rejected_reason{"not_evaluated"};
};

class PalmReferenceDetector {
public:
    PalmReferenceDetector();
    ~PalmReferenceDetector() = default;

    // Load empty reference images
    bool loadReference(int slot_id, const PalmReferenceSlotParams & params);

    // Evaluate current RGB + Depth frame
    PalmReferenceResult evaluate(
    int slot_id,
    const cv::Mat & rgb,
    const cv::Mat & depth,
    const cv::Rect & detect_roi);

private:
    struct ReferenceFrame {
        cv::Mat rgb_gray;
        cv::Mat depth;
    };

    std::map<int, ReferenceFrame> references_;
    std::map<int, PalmReferenceSlotParams> params_;
};

} // namespace weapon_tip_detector