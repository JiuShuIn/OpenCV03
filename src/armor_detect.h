// ============================================================
// 装甲板识别 —— 单帧检测接口
//
// 一张彩色图进，候选列表 + 调试图出。
// 不依赖任何全局变量，不写死坐标，换图走同一套流程。
// 下一阶段接视频时，逐帧调用 detectArmor 即可。
// ============================================================

#pragma once

#include "armor.h"
#include <vector>

// 单帧检测的全部输出
struct DetectResult {
    std::vector<LightBar> bars;      // 灯条候选
    std::vector<Armor>    armors;    // 装甲板候选
    cv::Mat mask;                    // 颜色 mask（单通道）
    cv::Mat barsImage;               // 候选图（画了灯条）
    cv::Mat resultImage;             // 结果图（画了装甲板）
    int contourCount = 0;            // 轮廓总数，调试用
};

// 对一张 BGR 图做完整检测
// 返回 false 表示输入非法（空图）或流程中断；此时 out 的内容不可用
bool detectArmor(const cv::Mat& bgr, const Params& p, DetectResult& out);
