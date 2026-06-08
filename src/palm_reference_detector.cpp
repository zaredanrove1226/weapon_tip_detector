#include "weapon_tip_detector/palm_reference_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>
#include <cmath>

namespace weapon_tip_detector {

PalmReferenceDetector::PalmReferenceDetector() {}

bool PalmReferenceDetector::loadReference(int slot_id, const PalmReferenceSlotParams & param)
{
    params_[slot_id] = param;

    ReferenceFrame ref;
    cv::Mat rgb = cv::imread(param.empty_rgb_path, cv::IMREAD_COLOR);
    if (rgb.empty()) return false;
    cv::cvtColor(rgb, ref.rgb_gray, cv::COLOR_BGR2GRAY);

    cv::Mat depth = cv::imread(param.empty_depth_path, cv::IMREAD_UNCHANGED);

    if (!depth.empty()) {
    if (depth.type() == CV_32FC1) {
        ref.depth = depth.clone();
    } else if (depth.type() == CV_16UC1) {
        depth.convertTo(ref.depth, CV_32FC1, 0.001);
    } else {
        ref.depth.release();
    }
    } else {
    ref.depth.release();
    }

    references_[slot_id] = ref;

    return true;
}

PalmReferenceResult PalmReferenceDetector::evaluate(
  int slot_id,
  const cv::Mat & rgb,
  const cv::Mat & depth,
  const cv::Rect & detect_roi)
{
    PalmReferenceResult result;

    if (references_.find(slot_id) == references_.end()) {
        result.rejected_reason = "No empty reference loaded";
        return result;
    }

    const auto & ref = references_[slot_id];
    const auto & param = params_[slot_id];

    cv::Mat rgb_gray;
    cv::cvtColor(rgb, rgb_gray, cv::COLOR_BGR2GRAY);

    const cv::Rect image_rect(0, 0, rgb_gray.cols, rgb_gray.rows);
    const cv::Rect roi = detect_roi & image_rect;

    if (roi.width <= 0 || roi.height <= 0) {
        result.rejected_reason = "invalid_detect_roi";
        return result;
    }

    if (rgb_gray.size() != ref.rgb_gray.size()) {
    result.rejected_reason = "reference_rgb_size_mismatch";
    return result;
    }

    result.rgb_diff_mask = cv::Mat::zeros(rgb_gray.size(), CV_8UC1);

    cv::Mat diff_roi;
    cv::absdiff(rgb_gray(roi), ref.rgb_gray(roi), diff_roi);

    cv::Mat dark_roi;
    cv::threshold(
    rgb_gray(roi),
    dark_roi,
    param.dark_gray_threshold,
    255,
    cv::THRESH_BINARY_INV);

    cv::Mat diff_binary_roi;
    cv::threshold(
    diff_roi,
    diff_binary_roi,
    param.rgb_diff_threshold,
    255,
    cv::THRESH_BINARY);

    cv::Mat new_dark_roi;
    cv::bitwise_and(dark_roi, diff_binary_roi, new_dark_roi);

    new_dark_roi.copyTo(result.rgb_diff_mask(roi));

    // Depth diff, current pipeline depth is expected to be CV_32FC1 in meters.
    cv::Mat depth_diff_mask = cv::Mat::zeros(rgb_gray.size(), CV_8UC1);
    cv::Mat depth_diff_roi = cv::Mat::zeros(roi.height, roi.width, CV_8UC1);

    if (!ref.depth.empty() && !depth.empty() &&
        ref.depth.size() == depth.size() &&
        ref.depth.type() == CV_32FC1 &&
        depth.type() == CV_32FC1)
    {
        const cv::Mat now_depth_roi = depth(roi);
        const cv::Mat ref_depth_roi = ref.depth(roi);

        for (int y = 0; y < roi.height; ++y) {
            const float * now_ptr = now_depth_roi.ptr<float>(y);
            const float * ref_ptr = ref_depth_roi.ptr<float>(y);
            uint8_t * mask_ptr = depth_diff_roi.ptr<uint8_t>(y);

            for (int x = 0; x < roi.width; ++x) {
                const float now_z = now_ptr[x];
                const float ref_z = ref_ptr[x];

                const bool now_valid = std::isfinite(now_z) && now_z > 0.05f;
                const bool ref_valid = std::isfinite(ref_z) && ref_z > 0.05f;

                const bool depth_nearer =
                    now_valid &&
                    ref_valid &&
                    static_cast<double>(now_z) <
                      static_cast<double>(ref_z) - param.depth_delta;

                const bool depth_hole =
                    param.allow_depth_hole &&
                    ref_valid &&
                    !now_valid;

                mask_ptr[x] = (depth_nearer || depth_hole) ? 255 : 0;
            }
        }

        depth_diff_roi.copyTo(depth_diff_mask(roi));
    }

    result.depth_diff_mask = depth_diff_mask;

    // Combine masks only inside detect_roi.
    cv::Mat candidate_roi;
    cv::bitwise_or(new_dark_roi, depth_diff_roi, candidate_roi);

    // Morphology to fill small gaps.
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(candidate_roi, candidate_roi, cv::MORPH_CLOSE, kernel);

    result.candidate_mask = cv::Mat::zeros(rgb_gray.size(), CV_8UC1);
    candidate_roi.copyTo(result.candidate_mask(roi));

    // Connected components only inside detect_roi.
    cv::Mat labels, stats, centroids;
    int num_components = cv::connectedComponentsWithStats(
        candidate_roi,
        labels,
        stats,
        centroids,
        8,
        CV_32S);

    result.raw_component_count = std::max(0, num_components - 1);

    int best_idx = -1;
    double best_score = 0.0;
    for (int i = 1; i < num_components; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        double aspect =
        static_cast<double>(width) /
        std::max(1.0, static_cast<double>(height));
        double fill_ratio =
        static_cast<double>(area) /
        std::max(1.0, static_cast<double>(width * height));

        if (area < param.min_area || width < param.min_width || height < param.min_height)
            continue;
        if (aspect < param.min_aspect_ratio) continue;
        if (fill_ratio < param.min_fill_ratio) continue;

        double score = area + aspect*10 + fill_ratio*100;
        if (score > best_score) {
            best_score = score;
            best_idx = i;
        }
    }

    if (best_idx >= 0) {
        result.raw_present = true;
        result.final_score = best_score;
        result.rejected_reason = "accepted_reference_diff";
        result.best_bbox = cv::Rect(
            roi.x + stats.at<int>(best_idx, cv::CC_STAT_LEFT),
            roi.y + stats.at<int>(best_idx, cv::CC_STAT_TOP),
            stats.at<int>(best_idx, cv::CC_STAT_WIDTH),
            stats.at<int>(best_idx, cv::CC_STAT_HEIGHT)
        );

        result.best_area = stats.at<int>(best_idx, cv::CC_STAT_AREA);
    } else {
        result.rejected_reason = "No candidate passes thresholds";
    }

    return result;
}

} // namespace weapon_tip_detector