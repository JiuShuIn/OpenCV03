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
图像: images/outpost_frame.png  宽=1280 高=1024 通道=3
轮廓总数 = 12
灯条候选 = 2 根
  [灯条 0] center=(544.5,719.5) len=31 wid=9 tilt=0
  [灯条 1] center=(621.5,712) len=32 wid=7 tilt=0
装甲板候选 = 1 块
[装甲板 0] color=RED box=(540,696,86,40) center=(582.5,715.75)
  corners: 0=(540,704) 1=(625,696) 2=(625,728) 3=(540,735)
  leftId=0 rightId=1 valid=1
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
| 面积 | `contourArea` | 60 ~ 400 | 噪点 ≤33，真灯条 172~193 |
| 长宽比 | 长边 / 短边（用 `minAreaRect`） | ≥ 2.0 | 真灯条 3.4~4.6，方形噪点 1.0~1.8 |
| 填充率 | `contourArea / (w*h)` | ≥ 0.4 | 真灯条 0.69~0.77 |
| 倾角 | 长轴与竖直方向夹角 | ≤ 60° | 排除接近水平的杂光 |

**倾角自己算，不用 `rect.angle`**：OpenCV 4.5 前后 `angle` 定义不同，且 0° 和 90° 都返回 90。改为从 `rect.points()` 取 4 个顶点，找最长边，用 `atan2` 求方向。保留**符号**，因为配对时需要知道两根灯条是否朝同一方向倾斜。

### 3. 配对

两根灯条两两组合，检查六个条件，全部用相对比例：

| 条件 | 表达式 | 阈值 |
| --- | --- | --- |
| 颜色一致 | `A.color == B.color` | |
| 长度相近 | `min(len) / max(len)` | ≥ 0.7 |
| 方向协调 | `abs(A.angle - B.angle)` | ≤ 10° |
| 高度偏差 | `dyOff / avgLen` | ≤ 0.4 |
| 间距下限 | `gap / avgLen` | ≥ 1.0 |
| 间距上限 | `gap / avgLen` | ≤ 3.0 |

**`gap` 和 `dyOff` 沿灯条自身方向分解**，不用图像坐标的 dx/dy：

```cpp
axis = (sin(tilt), cos(tilt))    // 灯条长轴方向
n    = (-axis.y, axis.x)         // 垂直方向
gap   = |v · n|                  // 垂直间距
dyOff = |v · axis|               // 沿灯条方向的高度差
```

目标倾斜时，图像坐标的 dy 会把「整块板倾斜」误判成「两根灯条高度不一致」，导致漏检。

左右判断：`center.x` 小的在左。去重：内层循环从 `i+1` 开始，每对只处理一次。

### 4. 四角点顺序

固定顺序：**左上 → 右上 → 右下 → 左下**。

不用「y 最小 / y 最大」排序，因为灯条窄、倾斜小时 y 值会并列，无法区分同一高度的两个点。

改用**象限法**：

1. 装甲板中心 `C` = 两根灯条中心的中点
2. 水平方向 `u` = 从左灯条指向右灯条（归一化）
3. 垂直方向 `v` = `u` 逆时针转 90°
4. 8 个顶点各自相对 `C` 求投影 `du`、`dv`，按正负分四个象限
5. 每个象限里离 `C` 最远的点，就是那个角

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
├── images_test/              # 自测用的旋转图（可选）
├── input/
│   └── Input_Video.avi
├── output/                   # 运行时生成
└── src/
    ├── armor.h               # 数据结构：Color / LightBar / Armor / Params
    ├── armor_detect.h        # 单帧检测接口
    ├── armor_detect.cpp      # 检测流程实现
    └── main.cpp              # 入口：读图、调用、打印、存图
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
