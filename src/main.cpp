// ============================================================
// 装甲板识别 —— 程序入口
// 职责只有四件事：读图 → 调用 detectArmor → 打印结果 → 存图
// 检测逻辑全部在 armor_detect.cpp，main 不参与任何判断
// ============================================================

#include "armor.h"
#include "armor_detect.h"

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <string>

using namespace std;

// 把 Color 枚举转成终端可读的文字
static string colorName(Color c) {
    switch (c) {
        case Color::RED:   return "RED";
        case Color::BLUE:  return "BLUE";
        default:           return "UNKNOWN";
    }
}

int main(int argc, char** argv) {
    // 命令行参数优先，否则用默认图（相对项目根目录）
    string path = (argc > 1) ? argv[1] : "images/outpost_frame.png";

    cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
    if (image.empty()) {
        cerr << "读取失败，路径：" << path << endl;
        return 1;
    }
    cout << "图像: " << path << "  宽=" << image.cols
         << " 高=" << image.rows << " 通道=" << image.channels() << endl;

    filesystem::create_directories("output");

    Params p;
    DetectResult result;
    if (!detectArmor(image, p, result)) {
        cerr << "检测失败" << endl;
        return 1;
    }

    // ---- 保存中间结果 ----
    if (!cv::imwrite("output/mask_red.png", result.mask)) {
        cerr << "保存 output/mask_red.png 失败" << endl;
        return 1;
    }
    if (!cv::imwrite("output/bars.png", result.barsImage)) {
        cerr << "保存 output/bars.png 失败" << endl;
        return 1;
    }
    if (!cv::imwrite("output/result.png", result.resultImage)) {
        cerr << "保存 output/result.png 失败" << endl;
        return 1;
    }

    // ---- 终端输出 ----
    cout << "轮廓总数 = " << result.contourCount << endl;
    cout << "灯条 = " << result.bars.size() << " 根" << endl;
    for (size_t k = 0; k < result.bars.size(); k++) {
        const LightBar& b = result.bars[k];
        cout << "  [灯条 " << b.id << "] center=(" << b.center.x << ","
             << b.center.y << ") len=" << b.length
             << " wid=" << b.width << " tilt=" << b.angle << endl;
    }

    cout << "装甲板 = " << result.armors.size() << " 块" << endl;
    if (result.armors.empty()) {
        cout << "未检测到装甲板" << endl;
    }
    for (size_t k = 0; k < result.armors.size(); k++) {
        const Armor& ar = result.armors[k];
        cout << "[装甲板 " << k << "] color=" << colorName(ar.color)
             << " box=(" << ar.box.x << "," << ar.box.y << ","
             << ar.box.width << "," << ar.box.height << ")"
             << " center=(" << ar.center.x << "," << ar.center.y << ")" << endl;
        cout << "  corners:";
        for (int m = 0; m < 4; m++) {
            cout << " " << m << "=(" << ar.corners[m].x << "," << ar.corners[m].y << ")";
        }
        cout << endl;
        cout << "  barA=" << ar.barAId << " barB=" << ar.barBId
             << " valid=" << (ar.valid ? 1 : 0) << endl;
    }

    cout << "已保存 output/mask_red.png, output/bars.png, output/result.png" << endl;
    return 0;
}
