#ifndef WEAPON_TIP_DETECTOR__DETECTOR_UTILS_HPP_
#define WEAPON_TIP_DETECTOR__DETECTOR_UTILS_HPP_

#include "sensor_msgs/msg/camera_info.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace weapon_tip_detector
{

double clamp01(double v);

bool validIntrinsics(const sensor_msgs::msg::CameraInfo & info);

double percentile(std::vector<float> values, double p);

double closenessScore(double value, double ideal, double tolerance);

cv::Mat orMaskSafe(const cv::Mat & a, const cv::Mat & b);

}  // namespace weapon_tip_detector

#endif  // WEAPON_TIP_DETECTOR__DETECTOR_UTILS_HPP_