// ============================================================
// 装甲板识别 —— 程序入口
//
// 支持两种输入：
//   视频：逐帧检测，把结果画在每一帧上，输出新视频（默认）
//   图片：检测一次，保存 mask / 候选图 / 结果图
// 检测逻辑全部在 armor_detect.cpp，main 只负责读写和显示
// ============================================================

#include "armor.h"
#include "armor_detect.h"
#include "pose.h"

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

// 把 Color 枚举转成可读文字
static string colorName(Color c) {
    switch (c) {
        case Color::RED:   return "RED";
        case Color::BLUE:  return "BLUE";
        default:           return "UNKNOWN";
    }
}

// 判断路径是不是视频文件（按扩展名）
static bool isVideoPath(const string& path) {
    string ext;
    size_t dot = path.find_last_of('.');
    if (dot != string::npos) ext = path.substr(dot);
    for (char& ch : ext) ch = (char)tolower(ch);
    return ext == ".mp4" || ext == ".avi" || ext == ".mov" ||
           ext == ".mkv" || ext == ".flv" || ext == ".wmv";
}

// 把终端要打印的信息画到画面右下角
// 黑底黄字，右对齐，从下往上排
static void drawInfoPanel(cv::Mat& img, const DetectResult& r) {
    // 组装文字行，和终端输出保持一致
    vector<string> lines;
    lines.push_back("contours = " + to_string(r.contourCount));
    lines.push_back("bars = " + to_string(r.bars.size()));
    lines.push_back("armors = " + to_string(r.armors.size()));
    for (size_t k = 0; k < r.armors.size(); k++) {
        const Armor& ar = r.armors[k];
        char buf[160];
        snprintf(buf, sizeof(buf), "[%zu] %s center=(%.1f,%.1f) barA=%d barB=%d",
                 k, colorName(ar.color).c_str(), ar.center.x, ar.center.y,
                 ar.barAId, ar.barBId);
        lines.push_back(buf);
    }
    if (r.armors.empty()) {
        lines.push_back("no armor");
    }

    const double fontScale = 0.6;
    const int thickness = 1;
    const int margin = 12;
    const int lineHeight = 22;

    // 从下往上排：最后一行贴近底边
    int y = img.rows - margin;
    for (int i = (int)lines.size() - 1; i >= 0; i--) {
        int base = 0;
        cv::Size ts = cv::getTextSize(lines[i], cv::FONT_HERSHEY_SIMPLEX,
                                      fontScale, thickness, &base);
        cv::Point org(img.cols - margin - ts.width, y);
        // 先画黑色粗体描边，再画黄色字，保证任何背景上都看得清
        cv::putText(img, lines[i], org, cv::FONT_HERSHEY_SIMPLEX,
                    fontScale, cv::Scalar(0, 0, 0), thickness + 2);
        cv::putText(img, lines[i], org, cv::FONT_HERSHEY_SIMPLEX,
                    fontScale, cv::Scalar(0, 255, 255), thickness);
        y -= lineHeight;
    }
}

// ------------------------------------------------------------
// 在结果图上标注位姿：每块装甲板中心上方显示距离
// poses 与 result.armors 一一对应
// ------------------------------------------------------------
static void drawPoseOverlay(cv::Mat& img,
                            const DetectResult& result,
                            const vector<PoseResult>& poses) {
    for (size_t k = 0; k < poses.size(); k++) {
        if (!poses[k].valid) continue;

        char buf[32];
        snprintf(buf, sizeof(buf), "%.2fm", poses[k].distance);

        // 写在装甲板中心上方，避开四角点编号
        cv::Point org((int)result.armors[k].center.x - 20,
                      (int)result.armors[k].center.y - 42);
        // 黑色描边 + 黄色字，任何背景都看得清
        cv::putText(img, buf, org, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 0, 0), 3);
        cv::putText(img, buf, org, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 255), 1);
    }
}

// 打印单帧检测结果到终端
// poses 与 result.armors 一一对应，记录每块装甲板的位姿
static void printResult(const DetectResult& result,
                        const vector<PoseResult>& poses) {
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

        // 位姿：成功了打印位置、距离、重投影误差
        const PoseResult& pr = poses[k];
        if (pr.valid) {
            cout << "  tvec=(" << pr.tvec[0] << ", " << pr.tvec[1] << ", "
                 << pr.tvec[2] << ") m" << endl;
            cout << "  distance=" << pr.distance << " m"
                 << "  reproj=" << pr.reprojError << " px" << endl;
        } else {
            cout << "  pose: 解算失败" << endl;
        }
    }
}

// ------------------------------------------------------------
// 图片模式：检测一次，存三张图
// ------------------------------------------------------------
static int runImage(const string& path) {
    cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
    if (image.empty()) {
        cerr << "读取失败，路径：" << path << endl;
        return 1;
    }
    cout << "图像: " << path << "  宽=" << image.cols
         << " 高=" << image.rows << " 通道=" << image.channels() << endl;

    Params p;
    DetectResult result;
    if (!detectArmor(image, p, result)) {
        cerr << "检测失败" << endl;
        return 1;
    }

    // ---- 位姿解算 ----
    // 每块装甲板调用一次 solvePose，结果和 armors 一一对应
    //         （结构体里已经有默认值，直接定义即可）
    CameraParams cam;

    vector<PoseResult> poses;
    for (size_t k = 0; k < result.armors.size(); k++) {
    poses.push_back(solvePose(result.armors[k].corners, cam));
    }

    drawPoseOverlay(result.resultImage, result, poses);

    // 结果图上叠加信息面板，和视频模式保持一致
    drawInfoPanel(result.resultImage, result);

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

    printResult(result, poses);
    cout << "已保存 output/mask_red.png, output/bars.png, output/result.png" << endl;
    return 0;
}

// ------------------------------------------------------------
// 视频模式：逐帧检测，结果画在每一帧上，写成新视频
// ------------------------------------------------------------
static int runVideo(const string& path, const string& outPath) {
    cv::VideoCapture cap(path);
    if (!cap.isOpened()) {
        cerr << "无法打开视频：" << path << endl;
        return 1;
    }

    int width  = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);
    int total  = (int)cap.get(cv::CAP_PROP_FRAME_COUNT);
    if (fps <= 1.0) fps = 30.0;   // 有些视频读不出帧率，给个默认值

    cout << "视频: " << path << "  " << width << "x" << height
         << "  fps=" << fps << "  总帧数=" << total << endl;

    // 编码器优先级：H.264(avc1) 浏览器/VS Code 能直接播，
    // mp4v 是备选（部分播放器不认）
    cv::VideoWriter writer;
    const int fourccs[] = {
        cv::VideoWriter::fourcc('a', 'v', 'c', '1'),
        cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
        cv::VideoWriter::fourcc('X', 'V', 'I', 'D')
    };
    const char* names[] = { "H.264", "MPEG-4", "XVID" };
    for (int i = 0; i < 3 && !writer.isOpened(); i++) {
        writer.open(outPath, fourccs[i], fps, cv::Size(width, height));
        if (writer.isOpened()) {
            cout << "使用编码器: " << names[i] << endl;
        }
    }
    if (!writer.isOpened()) {
        cerr << "无法创建输出视频：" << outPath << endl;
        return 1;
    }

    Params p;
    DetectResult result;
    CameraParams cam;         // 相机参数，结构体里已有默认值
    cv::Mat frame;
    int frameIndex = 0;
    int armorFrames = 0;      // 检测到装甲板的帧数
    bool savedFirst = false;  // 是否已保存第一帧的调试图

    // 每帧的位姿数据，最后一次性写 CSV（避免每帧开关文件）
    vector<string> csvLines;
    csvLines.push_back("frame_id,time_s,candidate_id,detect_valid,pnp_valid,"
                       "tx_m,ty_m,tz_m,distance_m,rx_rad,ry_rad,rz_rad,"
                       "reprojection_error_px");

    while (cap.read(frame)) {
        if (frame.empty()) break;

        if (!detectArmor(frame, p, result)) {
            cerr << "第 " << frameIndex << " 帧检测失败，跳过" << endl;
            frameIndex++;
            continue;
        }

        // 位姿解算：每块装甲板一次，和 armors 一一对应
        vector<PoseResult> poses;
        for (size_t k = 0; k < result.armors.size(); k++) {
            poses.push_back(solvePose(result.armors[k].corners, cam));
        }

        // 记录本帧每个候选的数据
        double timeSec = frameIndex / fps;
        if (result.armors.empty()) {
            // 没有候选也要留一行，detect_valid = 0
            char line[256];
            snprintf(line, sizeof(line), "%d,%.4f,,0,0,,,,,,,,",
                     frameIndex, timeSec);
            csvLines.push_back(line);
        } else {
            for (size_t k = 0; k < poses.size(); k++) {
                const PoseResult& pr = poses[k];
                char line[512];
                if (pr.valid) {
                    snprintf(line, sizeof(line),
                             "%d,%.4f,%zu,1,1,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f",
                             frameIndex, timeSec, k,
                             pr.tvec[0], pr.tvec[1], pr.tvec[2], pr.distance,
                             pr.rvec[0], pr.rvec[1], pr.rvec[2], pr.reprojError);
                } else {
                    snprintf(line, sizeof(line), "%d,%.4f,%zu,1,0,,,,,,,,",
                             frameIndex, timeSec, k);
                }
                csvLines.push_back(line);
            }
        }

        // 在结果图上叠加距离文字和信息面板
        drawPoseOverlay(result.resultImage, result, poses);
        drawInfoPanel(result.resultImage, result);

        // 第一帧额外保存 mask 和候选图，方便排查
        if (!savedFirst) {
            cv::imwrite("output/mask_red.png", result.mask);
            cv::imwrite("output/bars.png", result.barsImage);
            savedFirst = true;
        }

        writer.write(result.resultImage);
        if (!result.armors.empty()) armorFrames++;

        // 每 30 帧打印一次进度，避免刷屏
        if (frameIndex % 30 == 0) {
            cout << "帧 " << frameIndex << "/" << total
                 << "  bars=" << result.bars.size()
                 << "  armors=" << result.armors.size() << endl;
        }
        frameIndex++;
    }

    cap.release();
    writer.release();

    // 写 CSV
    const string csvPath = "output/pose.csv";
    FILE* fp = fopen(csvPath.c_str(), "w");
    if (fp) {
        for (size_t i = 0; i < csvLines.size(); i++) {
            fprintf(fp, "%s\n", csvLines[i].c_str());
        }
        fclose(fp);
        cout << "已保存 " << csvPath << "（" << csvLines.size() - 1 << " 行数据）" << endl;
    } else {
        cerr << "无法写入 " << csvPath << endl;
    }

    cout << "处理完成：共 " << frameIndex << " 帧，其中 "
         << armorFrames << " 帧检测到装甲板" << endl;
    cout << "已保存 " << outPath << "（以及第一帧的 output/mask_red.png、output/bars.png）" << endl;
    return 0;
}

int main(int argc, char** argv) {
    // 默认处理视频；传图片路径则走图片模式
    string path    = (argc > 1) ? argv[1] : "input/outpost.mp4";
    string outPath = (argc > 2) ? argv[2] : "output/result.mp4";

    filesystem::create_directories("output");

    if (isVideoPath(path)) {
        return runVideo(path, outPath);
    }
    return runImage(path);
}
