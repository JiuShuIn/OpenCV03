// ============================================================
// 位姿解算 —— 实现
//
// 一个函数：solvePose
// 输入 4 个图像角点 + 相机参数，输出装甲板的位置、朝向、距离、误差。
//
// 这里只做数学，不知道装甲板是怎么检测出来的。
// ============================================================

#include "pose.h"

#include <cmath>
#include <vector>

using namespace std;
// ------------------------------------------------------------
// solvePose —— 单块装甲板的位姿解算
//
// 内部 5 步：
//   1. 造 4 个三维模型点
//   2. 把 Point2f 角点转成 Point2d
//   3. solvePnP
//   4. 取 tvec 算直线距离
//   5. projectPoints 算重投影误差
// ------------------------------------------------------------
PoseResult solvePose(const cv::Point2f corners[4], const CameraParams& cam) {
    PoseResult result;   // valid = false，失败时直接返回它

    // ---- 第一步：造 4 个三维模型点 ----
    // 装甲板中心为原点，z = 0 平面。
    // 顺序必须和 corners 一致：左上、右上、右下、左下。
    //   w/2 = 0.0675   h/2 = 0.028
    // 用花括号 {} 构造 Point3d，不要用圆括号 ()
    vector<cv::Point3d> objectPoints = {
        {-ARMOR_WIDTH  / 2, -ARMOR_HEIGHT / 2, 0},   // 0 左上
        { ARMOR_WIDTH  / 2, -ARMOR_HEIGHT / 2, 0},   // 1 右上
        { ARMOR_WIDTH  / 2,  ARMOR_HEIGHT / 2, 0},   // 2 右下
        {-ARMOR_WIDTH  / 2,  ARMOR_HEIGHT / 2, 0}    // 3 左下
    };


    // ---- 第二步：把四个角点转成 solvePnP 要的类型 ----
    // corners 是 cv::Point2f（float），solvePnP 要 cv::Point2d（double）
    vector<cv::Point2d> imagePoints;
    for (int i = 0; i < 4; i++) { 
        imagePoints.push_back(cv::Point2d(corners[i].x, corners[i].y));
    }


    // ---- 第三步：调用 solvePnP ----
    // 原型：bool solvePnP(objectPoints, imagePoints, K, D, rvec, tvec)
    //   rvec、tvec 是输出，用 cv::Mat 接收（3x1）
    // 返回 false 时 rvec/tvec 内容不可用，必须检查
    // 第四个参数之后传方法标志：默认的 ITERATIVE 对共面 4 点会抛异常，
    // IPPE 专门用于平面目标，4 个点即可
    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(objectPoints, imagePoints, cam.K, cam.D, rvec, tvec,
                           false, cv::SOLVEPNP_IPPE);
    if (!ok) return result;   // result.valid 还是 false


    // ---- 第四步：从 tvec 取出三个数，算直线距离 ----
    // tvec 是 3x1 的 cv::Mat，用 .at<double>(i) 取第 i 行
    //   tx -> 左右，ty -> 上下，tz -> 前后深度
    // 距离不是 tz，而是三维直线长度
    // 还要检查 tz > 0 且三个数都是有限值，否则结果不可信
    double tx = tvec.at<double>(0);
    double ty = tvec.at<double>(1);
    double tz = tvec.at<double>(2);

    // 深度必须为正（装甲板在相机前方），且三个数都必须是有限值
    if (!std::isfinite(tx) || !std::isfinite(ty) || !std::isfinite(tz)) return result;
    if (tz <= 0.0) return result;

    result.tvec = cv::Vec3d(tx, ty, tz);
    result.rvec = cv::Vec3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
    result.distance = std::sqrt(tx * tx + ty * ty + tz * tz);
    result.valid = true;


    // ---- 第五步：重投影，算平均像素误差 ----
    // projectPoints(objectPoints, rvec, tvec, K, D, 输出)
    // 把模型点按解出来的位姿投回图像，逐个和 imagePoints 比距离
    // 误差 = 四个距离的平均值
    // 用 cv::norm(a - b) 算两点距离
    vector<cv::Point2d> reproj;
    cv::projectPoints(objectPoints, rvec, tvec, cam.K, cam.D, reproj);
    double err = 0;
    for (int i = 0; i < 4; i++) {
        err += cv::norm(reproj[i] - imagePoints[i]);
    }
    result.reprojError = err / 4.0;



    return result;
}
