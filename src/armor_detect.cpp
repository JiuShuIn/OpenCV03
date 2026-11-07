// ============================================================
// 装甲板识别 —— 单帧检测实现
//
// 本文件包含四个函数，按检测顺序排列：
//   makeColorMask  颜色分割：找出"颜色像灯条"的像素
//   filterBars     灯条筛选：从轮廓里挑出"形状像灯条"的
//   pairBars       灯条配对：判断哪两根是同一块装甲板
//   fillGeometry   几何计算：由两根灯条算出框、中心、四角点
// 最后由 detectArmor 把它们串起来。
// ============================================================

#include "armor_detect.h"

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

using namespace std;

// ------------------------------------------------------------
// makeColorMask —— 颜色分割
//
// 作用：把彩色图变成黑白图，白色 = 颜色符合灯条条件的像素。
// 输入：BGR 图像、阈值参数
// 输出：单通道二值图，尺寸与原图相同
// 后续：这张图交给 findContours，提取出一个个白色块
// ------------------------------------------------------------
static cv::Mat makeColorMask(const cv::Mat& bgr, const Params& p) {
    // BGR 转 HSV：H 对光照不敏感，且能用 S 排除白色照明板
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // 三通道同时落在区间内的像素置 255，其余置 0
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(p.hMin, p.sMin, p.vMin),
                     cv::Scalar(p.hMax, p.sMax, p.vMax), mask);

    // 开运算去掉小于 3x3 的噪点
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat cleaned;
    cv::morphologyEx(mask, cleaned, cv::MORPH_OPEN, kernel);
    return cleaned;
}

// ------------------------------------------------------------
// filterBars —— 灯条筛选
//
// 作用：mask 里每个白色块是一个轮廓，但大部分是噪点。
//       这里用四个几何条件，只留下"细长、竖直、大小合适"的。
// 输入：所有轮廓、阈值参数
// 输出：灯条候选列表，每个元素记录了位置、尺寸、倾角、颜色
// 条件：面积、长宽比、填充率、倾角，任一不满足就丢弃
// ------------------------------------------------------------
static vector<LightBar> filterBars(const vector<vector<cv::Point>>& contours,
                                   const Params& p) {
    vector<LightBar> bars;

    for (size_t i = 0; i < contours.size(); i++) {
        const vector<cv::Point>& c = contours[i];

        // 面积用多边形面积（不是像素个数）；旋转矩形贴合倾斜目标
        double area = cv::contourArea(c);
        cv::RotatedRect rect = cv::minAreaRect(c);

        float w = rect.size.width;
        float h = rect.size.height;
        float longSide  = max(w, h);
        float shortSide = min(w, h);

        // 长宽比判断"细长"；填充率判断轮廓有多填满外接矩形
        float aspect   = (shortSide > 0) ? longSide / shortSide : 0.0f;
        float rectArea = w * h;
        float fill     = (rectArea > 0) ? (float)area / rectArea : 0.0f;

        // 四个几何条件，任一不满足就丢弃
        if (area < p.minArea)     continue;
        if (area > p.maxArea)     continue;
        if (aspect < p.minAspect) continue;
        if (fill < p.minFill)     continue;

        // 从 4 个顶点算有符号倾角，不用 rect.angle
        cv::Point2f pts[4];
        rect.points(pts);

        // 找最长边：灯条细长，最长边就是它的走向
        float bestDx = 0.0f, bestDy = 0.0f, bestLen = -1.0f;
        for (int k = 0; k < 4; k++) {
            float dx = pts[(k + 1) % 4].x - pts[k].x;
            float dy = pts[(k + 1) % 4].y - pts[k].y;
            float len = sqrt(dx * dx + dy * dy);
            if (len > bestLen) {
                bestLen = len;
                bestDx = dx;
                bestDy = dy;
            }
        }

        // 统一成"指向下方"，消除 ±180° 歧义
        if (bestDy < 0) {
            bestDx = -bestDx;
            bestDy = -bestDy;
        }

        // 有符号倾角：正数顶端偏左，负数顶端偏右
        // atan2(dx, dy) 以竖直方向为参考，返回值乘 180/π 转成度
        float tiltSigned = atan2(bestDx, bestDy) * 180.0f / (float)CV_PI;
        if (fabs(tiltSigned) > p.maxTiltAngle) continue;

        LightBar bar;
        bar.rect   = rect;
        bar.center = rect.center;
        bar.length = longSide;
        bar.width  = shortSide;
        bar.angle  = tiltSigned;
        bar.color  = Color::RED;      // 当前只做红色分割
        bar.id     = (int)bars.size(); // id 等于 bars 下标，后续可直接 bars[id]
        bars.push_back(bar);
    }

    return bars;
}

// ------------------------------------------------------------
// pairBars —— 灯条配对
//
// 作用：灯条筛选只保证"单个东西像灯条"，不保证它和另一根属于同一块装甲板。
//       这里把所有灯条两两组合，用几何关系判断哪两根能组成装甲板。
// 输入：灯条候选列表、阈值参数
// 输出：装甲板候选列表，记录了颜色和左右灯条的编号
//
// 判断依据：
//   1. 间距（gapRatio）—— 中心连线长度约为灯条长度的 1~3 倍
//   2. 垂直度（angleFromPerp）—— 中心连线应与灯条垂直，偏离不超过 15°
// 所有条件用相对比例，目标远近变化时仍成立。
// ------------------------------------------------------------
static vector<Armor> pairBars(const vector<LightBar>& bars, const Params& p) {
    vector<Armor> armors;

    for (size_t i = 0; i < bars.size(); i++) {
        for (size_t j = i + 1; j < bars.size(); j++) {
            const LightBar& A = bars[i];
            const LightBar& B = bars[j];

            // 颜色必须一致（当前只检测红色，这条为将来红蓝共存预留）
            if (A.color != B.color) continue;

            // 平均长度后面当分母，把绝对像素换算成比例
            float avgLen = (A.length + B.length) / 2.0f;
            if (avgLen <= 0.0f) continue;

            // 两根灯条应大致等长
            float lenMin = min(A.length, B.length);
            float lenMax = max(A.length, B.length);
            float lenRatio = lenMin / lenMax;
            if (lenRatio < p.minLengthRatio) continue;

            // 两根灯条应大致平行
            float angleDiff = fabs(A.angle - B.angle);
            if (angleDiff > p.maxAngleDiff) continue;


            // ---- 判断两根灯条能否组成一块装甲板 ----
            // 两个几何关系：
            //   1. 中心连线的长度 / 平均灯条长度  -> 间距是否合理
            //   2. 中心连线与灯条的夹角            -> 是否接近 90°（两根灯条并排）

            // 两根灯条中心连线的向量和长度
            float dx = B.center.x - A.center.x;
            float dy = B.center.y - A.center.y;
            float dist = sqrt(dx * dx + dy * dy);

            // 间距比：连线长度是灯条长度的几倍
            float gapRatio = dist / avgLen;

            // 灯条长轴方向的单位向量（两根倾角取均值）
            float avgTiltRad = (A.angle + B.angle) * 0.5f * (float)CV_PI / 180.0f;
            float yAxisX = sin(avgTiltRad);
            float yAxisY = cos(avgTiltRad);

            // 连线在灯条方向上的投影长度 = |dx*axisX + dy*axisY|
            // 它除以连线总长，就是"连线与灯条夹角"的余弦值
            // 两根灯条并排时连线应该垂直于灯条，夹角接近 90°，余弦接近 0
            float alongProj = fabs(dx * yAxisX + dy * yAxisY);
            float cosAngle = (dist > 0.0f) ? alongProj / dist : 1.0f;

            // 垂直度检查：夹角偏离 90° 超过 maxAngleFromPerp 度就丢弃
            //   cos(90°-θ) = sin(θ)，用小角度近似 sinθ ≈ θ(弧度)
            float sinTilt = cosAngle;
            float angleFromPerp = asin(sinTilt > 1.0f ? 1.0f : sinTilt) * 180.0f / (float)CV_PI;
            if (angleFromPerp > p.maxAngleFromPerp) continue;

            if (gapRatio < p.minGapRatio)  continue;
            if (gapRatio > p.maxGapRatio)  continue;

            // 左右判断：用连线在"垂直于灯条"方向上的符号
            //   把灯条方向顺时针转 90° 得到朝右的方向
            float rightX =  yAxisY;
            float rightY = -yAxisX;
            float bPerp = dx * rightX + dy * rightY;

            Armor ar;
            ar.color = A.color;
            ar.valid = true;
            if (bPerp > 0) {
                ar.leftId  = A.id;
                ar.rightId = B.id;
            } else {
                ar.leftId  = B.id;
                ar.rightId = A.id;
            }
            armors.push_back(ar);
        }
    }

    return armors;
}

// ------------------------------------------------------------
// fillGeometry —— 几何计算
//
// 作用：由左右两根灯条的旋转矩形，算出装甲板的四个角点、中心、外接框。
// 输入：一块装甲板候选（已记录左右灯条编号）、灯条列表
// 输出：直接写进这块装甲板的 corners / center / box
//
// 四角点顺序固定：左上 → 右上 → 右下 → 左下（下一阶段 solvePnP 的输入约定）
//
// 怎么定四个角：
//   用装甲板自身的方向（而不是图像 x/y 轴）划分四个象限：
//     u = 从左灯条指向右灯条
//     v = u 转 90°
//   每个象限里离中心最远的点，就是那个角。
//   这样目标倾斜时顺序也不会乱。
// ------------------------------------------------------------
static bool fillGeometry(Armor& ar, const vector<LightBar>& bars) {
    // ---- 步骤 1：检查编号合法性 ----
    // 编号是 bars 的下标，越界说明数据有问题，直接放弃这块
    if (ar.leftId < 0 || ar.leftId >= (int)bars.size()) return false;
    if (ar.rightId < 0 || ar.rightId >= (int)bars.size()) return false;

    // ---- 步骤 2：取出左右两根灯条 ----
    // 用引用，不复制；后面只读
    const LightBar& L = bars[ar.leftId];
    const LightBar& R = bars[ar.rightId];

    // ---- 步骤 3：把两根灯条的 8 个顶点收集到一起 ----
    // 每根灯条是个旋转矩形，各有 4 个顶点。
    // 装甲板的四个角，一定出自这 8 个点里。
    //   all[0..3] = 左灯条的 4 个顶点
    //   all[4..7] = 右灯条的 4 个顶点
    cv::Point2f lp[4], rp[4];
    L.rect.points(lp);
    R.rect.points(rp);

    cv::Point2f all[8];
    for (int m = 0; m < 4; m++) {
        all[m]     = lp[m];
        all[m + 4] = rp[m];
    }

    // ---- 步骤 4：算装甲板中心 ----
    // 就是两根灯条中心的中点，后面用来划分四个象限
    cv::Point2f C((L.center.x + R.center.x) * 0.5f,
                  (L.center.y + R.center.y) * 0.5f);

    // ---- 步骤 5：定两条方向轴（跟着装甲板走，不跟图像走）----
    // u 轴：从左灯条指向右灯条，是装甲板的"横向"
    // 先算差值，再除以长度变成单位向量（长度为 1）
    // 灯条中心重合时长度为 0，除零会得到 nan，所以先检查
    float ux = R.center.x - L.center.x;
    float uy = R.center.y - L.center.y;
    float ulen = sqrt(ux * ux + uy * uy);
    if (ulen < 1e-6f) return false;
    ux /= ulen;
    uy /= ulen;

    // v 轴：u 逆时针转 90°，是装甲板的"纵向"
    // 二维向量转 90° 的公式：(x, y) -> (-y, x)
    float vx = -uy;
    float vy =  ux;

    // ---- 步骤 6：准备四个"最佳角点"的记录 ----
    // i?? = 8 个顶点里的下标，-1 表示还没找到
    // d?? = 目前该象限里最远的距离，-1 是初始值（任何距离都比它大）
    int iLT = -1, iRT = -1, iRB = -1, iLB = -1;
    float dLT = -1.0f, dRT = -1.0f, dRB = -1.0f, dLB = -1.0f;

    // ---- 步骤 7：把 8 个顶点分到四个象限，每个象限留最远的那个 ----
    // 对每个顶点：
    //   1. 算它相对中心 C 的偏移 (px, py)
    //   2. 把偏移投影到 u、v 两轴上，得到 du、dv
    //   3. du 的正负决定在左还是右，dv 的正负决定在上还是下
    //   4. 同一象限里，离中心越远的点越可能是装甲板的角
    //
    //   du<0  dv<0  → 左上    du>0  dv<0  → 右上
    //   du<0  dv>0  → 左下    du>0  dv>0  → 右下
    for (int m = 0; m < 8; m++) {
        float px = all[m].x - C.x;
        float py = all[m].y - C.y;
        float du = px * ux + py * uy;   // 水平方向投影
        float dv = px * vx + py * vy;   // 竖直方向投影
        float dist = sqrt(px * px + py * py);

        if (du < 0 && dv < 0 && dist > dLT) { dLT = dist; iLT = m; }
        if (du > 0 && dv < 0 && dist > dRT) { dRT = dist; iRT = m; }
        if (du > 0 && dv > 0 && dist > dRB) { dRB = dist; iRB = m; }
        if (du < 0 && dv > 0 && dist > dLB) { dLB = dist; iLB = m; }
    }

    // ---- 步骤 8：检查四个象限都找到了点 ----
    // 任一象限为空，说明 8 个点的分布不符合"两块矩形拼成一块板"，放弃
    if (iLT < 0 || iRT < 0 || iRB < 0 || iLB < 0) return false;

    // ---- 步骤 9：按固定顺序写入四个角点 ----
    // 顺序：左上 → 右上 → 右下 → 左下（绕一圈）
    // 这是下一阶段 solvePnP 的输入约定，顺序错了会导致距离和朝向异常
    ar.corners[0] = all[iLT];
    ar.corners[1] = all[iRT];
    ar.corners[2] = all[iRB];
    ar.corners[3] = all[iLB];

    // ---- 步骤 10：算中心点 ----
    // 四个角点的平均位置（乘 0.25 等于除以 4）
    ar.center.x = (ar.corners[0].x + ar.corners[1].x + ar.corners[2].x + ar.corners[3].x) * 0.25f;
    ar.center.y = (ar.corners[0].y + ar.corners[1].y + ar.corners[2].y + ar.corners[3].y) * 0.25f;

    // ---- 步骤 11：算外接框 ----
    // 能包住四个角点的、边平行于图像坐标轴的矩形
    vector<cv::Point2f> pts4 = { ar.corners[0], ar.corners[1], ar.corners[2], ar.corners[3] };
    ar.box = cv::boundingRect(pts4);
    return true;
}

// ------------------------------------------------------------
// detectArmor —— 单帧检测主流程
//
// 作用：把上面四个函数串成一条流水线，一次处理一张图。
// 输入：一张 BGR 图、阈值参数
// 输出：DetectResult，含灯条列表、装甲板列表、三张调试图
// 返回：false 表示输入为空或流程中断
//
// 不依赖任何全局变量，不写死坐标，换图走同一套流程。
// 下一阶段接视频时，逐帧调用这个函数即可。
// ------------------------------------------------------------
bool detectArmor(const cv::Mat& bgr, const Params& p, DetectResult& out) {
    if (bgr.empty()) return false;

    // 1. 颜色分割
    out.mask = makeColorMask(bgr, p);

    // 2. 轮廓提取
    cv::Mat contourInput = out.mask.clone();   // findContours 会修改输入
    vector<vector<cv::Point>> contours;
    cv::findContours(contourInput, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    out.contourCount = (int)contours.size();

    // 3. 灯条筛选
    out.bars = filterBars(contours, p);

    // 4. 配对
    out.armors = pairBars(out.bars, p);

    // 5. 几何计算，失败的标记为无效并丢弃
    vector<Armor> validArmors;
    for (size_t k = 0; k < out.armors.size(); k++) {
        if (fillGeometry(out.armors[k], out.bars)) {
            validArmors.push_back(out.armors[k]);
        }
    }
    out.armors = validArmors;

    // 6. 调试图：候选灯条
    out.barsImage = bgr.clone();
    for (size_t k = 0; k < out.bars.size(); k++) {
        const LightBar& b = out.bars[k];
        cv::Point2f bp[4];
        b.rect.points(bp);
        for (int m = 0; m < 4; m++) {
            cv::line(out.barsImage, bp[m], bp[(m + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }
        cv::circle(out.barsImage, b.center, 3, cv::Scalar(0, 0, 255), -1);
        cv::putText(out.barsImage, to_string(b.id),
                    cv::Point((int)b.center.x - 8, (int)b.center.y - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    }

    // 7. 调试图：装甲板结果
    out.resultImage = bgr.clone();
    for (size_t k = 0; k < out.armors.size(); k++) {
        const Armor& ar = out.armors[k];
        cv::rectangle(out.resultImage, ar.box, cv::Scalar(0, 255, 0), 2);
        for (int m = 0; m < 4; m++) {
            cv::line(out.resultImage, ar.corners[m], ar.corners[(m + 1) % 4],
                     cv::Scalar(0, 255, 255), 1);
            cv::circle(out.resultImage, ar.corners[m], 4, cv::Scalar(255, 0, 255), -1);
            cv::putText(out.resultImage, to_string(m),
                        cv::Point((int)ar.corners[m].x + 6, (int)ar.corners[m].y - 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        }
        cv::circle(out.resultImage, ar.center, 5, cv::Scalar(0, 0, 255), -1);
    }

    return true;
}
