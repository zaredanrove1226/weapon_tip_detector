#include "weapon_tip_detector/detector_profiles.hpp"

namespace weapon_tip_detector
{

TipProfile defaultSpearProfile()
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

TipProfile defaultSpearHeadProfile()
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

  p.enable_ignore_mask = false;
  p.ignore_rects.clear();

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

TipProfile defaultSpearStemProfile()
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

TipProfile defaultFistProfile()
{
  TipProfile p;
  p.type = "fist";
  p.candidate_mask_mode = "foreground_or_dark";
  p.foreground_min_depth_diff = 0.010;
  p.min_component_area = 18;
  p.min_candidate_score = 0.25;
  p.max_component_area_ratio = 0.40;
  p.require_depth_for_candidate = false;
  p.enable_depth_behind_veto = true;
  p.depth_behind_veto_min_count = 120;
  p.depth_behind_veto_max_diff = -0.003;
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

TipProfile defaultFistBodyProfile()
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

TipProfile defaultFistStemProfile()
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

TipProfile defaultPalmProfile()
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

}  // namespace weapon_tip_detector