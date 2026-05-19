# current_tip_detector 重构前基线记录

## 目的

本记录用于 current_tip_detector_node.cpp 第一阶段低风险结构拆分后的行为对比。

第一阶段只拆分代码结构，不修改检测算法、不修改 profile 参数、不修改 ROI 逻辑、不修改 stable 判定逻辑。

---

## 场景说明

- 相机已安装到真机上，不能再像早期调试一样随意手持相机靠近端头。
- 当前截图来自重构前已有调试结果。
- 场地仍然是墙边调试环境，不代表交流赛无墙标准场地。
- 这些截图只用于验证“重构前后行为是否一致”。

---

## slot 1 spear：固定视角识别成功

截图：

`slot1_spear_ok.png`

观察结果：

- slot: 1
- expected: spear
- present: YES
- raw: YES
- stable: ON
- profile: spear_head
- candidate mask mode: foreground_or_dark
- best: accepted
- score: 约 0.77 / 0.300
- 现象：spear 在固定视角下可稳定识别，ROI 覆盖较好，目标主体明显。
- 重构要求：spear 结果不能被 fist/palm 的调整影响。

结论：

spear 是当前最稳定的基准样例。第一阶段重构后，slot 1 仍应保持 present YES / raw YES。

---

## slot 2 fist：手拿近后识别成功

截图：

`slot2_fist_ok_hand_close.png`

观察结果：

- slot: 2
- expected: fist
- present: YES
- raw: YES
- stable: ON
- profile: fist_body
- candidate mask mode: foreground_and_dark
- best: accepted
- score: 约 0.66 / 0.300
- 现象：手拿近后 fist_body 能形成有效候选，说明 fist 检测链路本身可用。

结论：

fist 算法不是完全失效，主要问题是固定视角下目标变小、ROI 和 mask 条件不适配。第一阶段重构后，手拿近的历史行为应保持可识别。

---

## slot 3 palm：手拿近后识别成功

截图：

`slot3_palm_ok_hand_close.png`

观察结果：

- slot: 3
- expected: palm
- present: YES
- raw: YES
- stable: ON
- profile: palm
- candidate mask mode: foreground_and_dark
- best: accepted
- score: 约 0.71 / 0.420
- 现象：手拿近后 palm 可以识别，说明 palm profile 不是完全错误。
- ROI 中目标横向主体明显，candidate 有效。

结论：

palm 的检测链路可用，但固定视角下更依赖 ROI 和 foreground mask。

---

## slot 2 fist：固定视角识别失败

截图：

`slot2_fist_fail_fixed_camera.png`

观察结果：

- slot: 2
- expected: fist
- present: NO
- raw: NO
- stable: ON
- profile: fist_stem_no_body
- candidate mask mode: foreground_and_dark
- dist: 0
- fg: 0
- dark: 约 8882
- cond: 0
- best: none
- 现象：固定视角下虽然 dark 有像素，但 foreground 为 0，foreground_and_dark 导致 candidate 为 0。
- fist_stem_no_body 不能单独确认，这是合理的防误检逻辑。

结论：

第一阶段重构后，slot 2 固定视角仍可能是 NO，这是旧行为，不应误认为重构失败。后续第二阶段再针对 ROI 和 candidate_mask_mode 优化。

---

## slot 3 palm：固定视角识别失败

截图：

`slot3_palm_fail_fixed_camera.png`

观察结果：

- slot: 3
- expected: palm
- present: NO
- raw: NO
- stable: ON
- profile: palm
- candidate mask mode: foreground_and_dark
- dist: 0
- fg: 0
- dark: 约 805
- cond: 0
- best: none
- 现象：固定视角下 palm 主体在 ROI 下方，ROI 偏上；同时 fg=0，foreground_and_dark 导致 candidate 为 0。

结论：

palm 固定视角失败主要是 ROI 和 mask 条件问题。第一阶段重构不解决该问题，只保证行为不被破坏。

---

## 第一阶段重构验收标准

重构后不要求算法变好，只要求：

1. 节点能正常启动。
2. `/weapon_tip_detector/current_present` 正常发布。
3. OpenCV preview 正常显示。
4. slot 1 spear 仍能识别。
5. fist/palm 的旧失败现象不因为拆分而变成异常崩溃。
6. 日志字段仍然能显示：
   - slot
   - expected
   - present
   - raw
   - stable
   - profile
   - cond_mode
   - dist / fg / dark / cond
   - best
   - score
   - bbox