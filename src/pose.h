// ============================================================
// 位姿解算 —— 类型定义
//
// 只放类型和常量，不写实现。
// 实现放在 pose.cpp，调用方是 main.cpp。
//
// 这一层的作用：把"装甲板在三维空间里在哪、朝哪"这件事
// 用一个结构体表达出来，和检测部分彻底解耦。
// ============================================================

#pragma once

#include <opencv2/opencv.hpp>

// 装甲板实际尺寸，单位米。
// 只有这个尺寸是对的，solvePnP 才能反推出以米为单位的距离。
constexpr double ARMOR_WIDTH  = 0.135;
constexpr double ARMOR_HEIGHT = 0.056;


// 相机标定参数，来源 agent/参数.md。
// 换相机只改这一处。width/height 用于检查输入帧尺寸是否和标定匹配。
struct CameraParams {
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        1781.053454, 0,           510.5751561,
        0,           1781.686873, 472.9735763,
        0,           0,           1);
    cv::Mat D = (cv::Mat_<double>(1, 5) <<
        -0.04420740074, -0.1081537096, -0.0006506219039,
         0.0002357380215, 0.466737247);
    int width  = 1280;
    int height = 1024;
};


// 一块装甲板的位姿结果。
// tvec 是位置（米），rvec 是朝向（弧度），distance 是到光心的直线距离。
struct PoseResult {
    bool      valid = false;
    cv::Vec3d tvec  = {0, 0, 0};
    cv::Vec3d rvec  = {0, 0, 0};
    double    distance    = 0.0;
    double    reprojError = 0.0;
};
// 由四个图像角点解算装甲板位姿
// corners 顺序必须是 左上 → 右上 → 右下 → 左下
PoseResult solvePose(const cv::Point2f corners[4], const CameraParams& cam);
