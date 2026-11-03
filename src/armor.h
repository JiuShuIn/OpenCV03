// ============================================================
// 装甲板识别 —— 数据结构定义
// 只放当前阶段要用到的类型，后续阶段需要时再补
// ============================================================

#pragma once   // 保证本头文件在一次编译中只展开一次，避免重复定义

#include <opencv2/opencv.hpp>



struct Params{
    // ---- 颜色分割：HSV 三通道范围 ----
    int hMin = 0;      // H 下限（色相，0~179）。0~35 覆盖红→橙
    int hMax = 35;     // H 上限
    int sMin = 100;    // S 下限（饱和度）。白板 S≈0，靠这个排除白色照明板
    int sMax = 255;    // S 上限
    int vMin = 100;    // V 下限（亮度）。暗背景 V 低，靠这个排除
    int vMax = 255;    // V 上限

    // ---- 灯条筛选：对每个轮廓逐个判断，任一不满足就丢弃 ----
    float minArea = 60.0f;        // 面积下限。噪点面积 ≤35，真灯条 172~193，取中间偏下
    float maxArea = 400.0f;       // 面积上限。排除大面积背景结构
    float minAspect = 2.0f;       // 长宽比下限（长边/短边）。灯条细长 3.4~4.6，方形噪点 1.0~1.8
    float minFill = 0.4f;         // 填充率下限（轮廓面积/矩形面积）。真灯条 0.69~0.77
    float maxTiltAngle = 60.0f;   // 倾角上限（长轴与竖直方向夹角）。只排除接近水平的杂光，不限制目标自身倾斜

    // ---- 灯条配对：两两组合，全部是相对比例，目标远近变化时仍成立 ----
    float minLengthRatio = 0.7f;  // 长度比下限（短/长）。两根灯条应大致等长
    float maxAngleDiff = 10.0f;   // 角度差上限（度）。两根灯条应大致平行
    float maxDyRatio = 0.4f;      // 高度偏差上限（沿灯条方向的距离/平均长度）。应大致在同一水平线
    float minGapRatio = 1.0f;     // 间距下限（垂直灯条方向的距离/平均长度）。排除贴太近的
    float maxGapRatio = 3.0f;     // 间距上限。排除离太远、不相关的两个亮点
    float maxAngleFromPerp = 15.0f; // 中心连线与灯条的夹角偏离 90° 的上限（度）。两根灯条应并排
};

// 用 enum class 而非 int/string：取值必须写 Color::RED，拼错会编译报错
enum class Color{
    RED,
    BLUE,
    UNKNOWN
};

// 一根灯条候选，阶段 4 产生，阶段 5 配对用
struct LightBar{
    cv::RotatedRect rect;   // 旋转矩形，阶段 6 取四顶点拼装甲板四角点
    cv::Point2f center;     // 中心点，阶段 5 算两灯条间距和高度差
    float length;           // 长边，阶段 5 做比例分母（dx/length）
    float width;            // 短边，阶段 4 算长宽比
    float angle;            // 长轴与竖直方向夹角，阶段 5 判平行
    Color color;            // 阶段 5 配对要求颜色一致
    int id;                 // 编号，结果图标注与配对核对用
};

// 一块装甲板候选，阶段 5 产生，阶段 6 补全几何量
struct Armor{
    Color       color;        // 颜色，取自组成它的两根灯条
    cv::Rect    box;          // 轴对齐外接框，阶段 6 算
    cv::Point2f center;       // 中心点，阶段 6 算
    cv::Point2f corners[4];   // 左上 → 右上 → 右下 → 左下，阶段 6 算
    int         leftId;       // 左灯条在 bars 里的下标
    int         rightId;      // 右灯条在 bars 里的下标
    bool        valid;        // 是否有效
};