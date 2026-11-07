# 装甲板识别（静态图片）

用传统视觉方法从前哨站图片中自动找出装甲板。流程：颜色分割 → 轮廓提取 → 灯条筛选 → 两两配对 → 几何计算。

## 环境要求

- CMake ≥ 3.16
- C++17 编译器（本机 g++ 15.2.0）
- OpenCV 4.x（本机 4.10.0）

Ubuntu 安装 OpenCV：

```bash
sudo apt install libopencv-dev
```

## 编译

在项目根目录执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

生成的可执行文件是 `build/armor_detect`。

## 运行

```bash
./build/armor_detect                              # 用默认图 images/outpost_frame.png
./build/armor_detect images/outpost_frame.png     # 显式指定路径
```

**必须在项目根目录运行**，因为程序用相对路径读写文件。

### 输出文件

运行后在 `output/` 下生成三张图：

| 文件 | 内容 |
| --- | --- |
| `mask_red.png` | 颜色分割后的二值 mask，白色是疑似目标像素 |
| `bars.png` | 灯条候选图，绿框标出每根候选并编号 |
| `result.png` | 最终结果图，绿框是外接框，紫点是四角点（标注 0/1/2/3），红点是中心 |

### 终端输出

```
图像: images/outpost_frame.png  宽=1280 高=1023 通道=3
轮廓总数 = 12
灯条 = 2 根
  [灯条 0] center=(657.5,310) len=32 wid=7 tilt=0
  [灯条 1] center=(734.5,302.5) len=31 wid=9 tilt=0
装甲板 = 1 块
[装甲板 0] color=RED box=(654,287,86,40) center=(696.5,306.25)
  corners: 0=(654,294) 1=(739,287) 2=(739,318) 3=(654,326)
  barA=0 barB=1 valid=1
已保存 output/mask_red.png, output/bars.png, output/result.png
```

没有候选时输出「未检测到装甲板」，退出码仍为 0。

## 方法说明

### 1. 颜色分割

选 **HSV + `inRange`**，没有用 BGR 通道差或灰度阈值。三种方法在同一张图上实测对比：

| 方法 | 白像素数 | 问题 |
| --- | --- | --- |
| HSV `inRange(0,100,100)-(35,255,255)` | 1582 | 灯条完整，白板被 S 下限排除 |
| `R - B > 60` | 1707 | 多了墙面暖色反光 |
| 灰度 >120 且 `R-B>60` | 803 | 灯条变细、有断开 |

选 HSV 的理由：

1. 白色照明板 V 极高但 S 接近 0，设 `Smin = 100` 一步排除；通道差只能间接排除
2. 色相 H 对光照变化不敏感，V 可以放宽
3. 红色跨色相边界的问题在 HSV 里是显式的，容易处理

**参数来源**：先在指定图的灯条区域实测，得到 `H 3~31, S 122~223, V 134~255`（10%~90% 分位），再放宽为 `H 0~35, S 100~255, V 100~255`。

**关于两段范围**：红色在色相环上跨 0 和 179 两端。本图灯条 H 实测 3~31，不跨边界，单段 `inRange` 即可覆盖，因此没有写第二段。

**蓝色**：本图只有橙红目标，未保留蓝色检测。

**形态学**：3×3 矩形核开运算，去掉小于核的噪点。核不能太大，否则两根灯条会粘连。

### 2. 灯条筛选

对每个轮廓计算四个几何量：

| 量 | 计算 | 阈值 | 依据 |
| --- | --- | --- | --- |
| 面积 | `contourArea` | 60 ~ 400 | 噪点 ≤33，真灯条 172~193；上限排除大面积背景结构 |
| 长宽比 | 长边 / 短边（用 `minAreaRect`） | ≥ 2.0 | 真灯条 3.4~4.6，方形噪点 1.0~1.8 |
| 填充率 | `contourArea / (w*h)` | ≥ 0.4 | 真灯条 0.69~0.77 |
| 倾角 | 长轴与竖直方向夹角 | ≤ 60° | 只排除接近水平的杂光（如墙面反光 tilt≈90），不限制目标自身倾斜 |

**倾角自己算，不用 `rect.angle`**：OpenCV 4.5 前后 `angle` 定义不同，且 0° 和 90° 都返回 90。改为从 `rect.points()` 取 4 个顶点，找最长边，用 `atan2` 求方向。保留**符号**，因为配对时需要知道两根灯条是否朝同一方向倾斜。

### 3. 配对

两根灯条两两组合，检查五个条件：

| 条件 | 表达式 | 阈值 |
| --- | --- | --- |
| 颜色一致 | `A.color == B.color` | |
| 长度相近 | `min(len) / max(len)` | ≥ 0.7 |
| 方向协调 | `abs(A.angle - B.angle)` | ≤ 10° |
| 间距 | `dist / avgLen` | 1.0 ~ 3.0 |
| 垂直度 | 连线与灯条夹角偏离 90° | ≤ 15° |

**间距用中心连线的欧氏距离**：

```cpp
float dx = B.center.x - A.center.x;
float dy = B.center.y - A.center.y;
float dist = sqrt(dx*dx + dy*dy);
float gapRatio = dist / avgLen;
```

欧氏距离是旋转不变量——目标怎么转，两根灯条之间的距离都不变，所以这条判断天然不受倾斜影响。

**垂直度检查**：两根并排的灯条，中心连线应该垂直于灯条本身。

```cpp
// 灯条长轴方向（两根倾角取均值）
float yAxisX = sin(avgTiltRad);
float yAxisY = cos(avgTiltRad);

// 连线在灯条方向上的投影
float alongProj = fabs(dx * yAxisX + dy * yAxisY);

// 投影占连线总长的比例 = 连线与灯条夹角的余弦
float cosAngle = alongProj / dist;
float angleFromPerp = asin(cosAngle) * 180.0f / CV_PI;
```

`alongProj / dist` 是「连线有多少比例落在灯条方向上」。连线完全垂直于灯条时，这个值是 0，`asin(0) = 0`，表示偏离垂直 0°。

实测四张图（原图 + 三张旋转图）都在 5° 左右，阈值取 15° 留了余量。

**去重**：内层循环从 `i+1` 开始，每对只处理一次。

### 4. 四角点顺序

固定顺序：**左上 → 右上 → 右下 → 左下**。

不用「y 最小 / y 最大」排序，因为灯条窄、倾斜小时 y 值会并列，无法区分同一高度的两个点。

改用**象限法**：

1. 装甲板中心 `C` = 两根灯条中心的中点
2. 纵向轴 `v` = 两根灯条倾角的均值，换算成单位向量 `(sin, cos)`
3. 横向轴 `u` = `v` 顺时针转 90°，即 `(v.y, -v.x)`
4. 8 个顶点各自相对 `C` 求投影 `du`、`dv`，按正负分四个象限
5. 每个象限里离 `C` 最远的点，就是那个角

倾角均值满足交换律，`(A+B)/2 == (B+A)/2`，所以两根灯条谁先谁后不影响坐标轴方向。配对阶段不需要判断左右，`Armor` 只记录两根灯条的编号 `barAId`、`barBId`。

用装甲板自身的方向而不是图像坐标轴，目标倾斜时顺序也不会乱。

这个顺序是下一阶段 `solvePnP` 的输入约定，顺序错了会导致距离和朝向异常。

## 项目结构

```
OpenCV03/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── 问题记录.md
├── images/
│   └── outpost_frame.png
├── images_test/              # 自测用的旋转图（不提交）
├── input/
│   └── Input_Video.mp4       # 原始视频（本阶段不使用）
├── output/                   # 运行时生成
└── src/
    ├── armor.h               # 数据结构：Color / LightBar / Armor / Params
    ├── armor_detect.h        # 单帧检测接口
    ├── armor_detect.cpp      # 检测流程实现
    └── main.cpp              # 入口：读图、调用 detectArmor、打印、存图
```

## 接口说明

下一阶段接视频时，逐帧调用这个函数：

```cpp
bool detectArmor(const cv::Mat& bgr, const Params& p, DetectResult& out);
```

- 输入：任意尺寸的 BGR 图像
- 输出：`DetectResult`，含灯条候选、装甲板候选、三张调试图
- 函数内不依赖全局变量，不写死任何坐标
- 换图走同一套流程，返回 0 个、1 个或多个候选

## 换图需要做什么

颜色参数依赖具体场景。换一张图时：

1. 在灯条区域取一小块，统计 H/S/V 的 10%~90% 分位
2. 把结果放宽后填进 `Params::hMin/hMax/sMin/sMax/vMin/vMax`
3. 重新运行，看 `output/mask_red.png` 是否只保留目标

几何阈值（面积、长宽比、配对比例）是相对量，通常不需要改；如果目标大小差异很大，再按实测调整。

## 自测

项目内附了 3 张旋转图（`images_test/`），用于验证倾斜情况下是否仍能正确检测：

```bash
./build/armor_detect images_test/rot_15.png
./build/armor_detect images_test/rot_-20.png
./build/armor_detect images_test/rot_30.png
```

三张图都应输出 1 块装甲板。
