# 装甲板识别与位姿解算

用传统视觉方法从前哨站画面中找出装甲板，并计算它相对相机的**三维位置和朝向**。

支持视频逐帧处理和单张图片处理。

数据链：

```
读取视频 → 逐帧检测装甲板 → 四角点 → solvePnP → 位置(tvec) + 朝向(rvec)
                                                    ↓
                                          projectPoints 重投影验证
                                                    ↓
                                          标注视频 + 逐帧 CSV
```

## 结果可信度规则（设计说明，未写入代码）

本阶段没有在程序里加"可信/可疑"判断逻辑，代码对所有解算成功的结果一视同仁地记录。
下面这条规则是设计产物，用于事后筛选数据、解释异常，也写在文档里说明数据边界。

### 规则

一条位姿记录判为**可信**，需同时满足：

| 条件 | 判据 | 依据 |
| --- | --- | --- |
| PnP 成功 | `pnp_valid == 1` | `solvePnP` 失败时 `tvec/rvec` 未定义，读取即为垃圾值 |
| 深度为正 | `tz_m > 0` | 装甲板必须在相机前方；`tz <= 0` 说明角点顺序或模型坐标系出错 |
| 数值有限 | 三个 tvec 分量均非 `nan`/`inf` | 角点退化（重合、共线）时会出现非有限值 |
| 重投影误差 | `reprojection_error_px < 3.0` | 见下方阈值依据 |

不满足任一条，判为**可疑**。

### 重投影误差阈值 3.0 px 的依据

| 数据 | 数值 |
| --- | --- |
| 完整视频（2220 个有效位姿）的误差中位数 | 0.98 px |
| 完整视频的最大值 | 2.80 px |
| 相机标定本身的重投影误差（参数.md 给出） | 0.33 px |

取 3.0 的原因：

- 略高于实测最大值 2.80，不误伤正常帧
- 远低于"角点错序"时的典型误差（数十像素），能拦住结构性问题
- 约为标定误差的 9 倍，留出足够余量

### 这条规则拦不住什么（漏报）

- **相机参数整体偏差**：`K`/`D` 有系统性错误时，`solvePnP` 会调整位姿让投影依然吻合，重投影误差仍然很小，但绝对距离整体偏移
- **装甲板尺寸写错**：同上，模型尺寸错 5% 会让距离偏 2~5%，重投影几乎不受影响（见下方敏感性实验）
- **恰好构成矩形的部分遮挡**：遮挡后残存的角点若仍能构成合理四边形，无法从几何上识别

### 这条规则会误伤什么（误报）

- **快速运动 + 视频压缩**：目标高速移动时边缘模糊，角点精度下降，重投影误差偏大但位姿可能仍然合理
- **画面极边缘的目标**：畸变模型在外圈误差增大，校正后角点与理想矩形有偏差

### 为什么没写进代码

1. 任务书只要求"设计一条规则"，并说明依据，不强制在程序中体现
2. 本阶段不做跟踪和预测，不存在"因为可疑所以丢弃当前帧"的需求，记录全部结果更利于事后分析
3. 阈值 3.0 是基于当前视频的实测统计，换场景需要重新标定；硬编码在代码里反而容易误导

需要时可在 CSV 里按上述条件筛选，或在使用 `PoseResult` 的地方加判断。

---

## 环境要求

- CMake ≥ 3.16
- C++17 编译器（开发环境 g++ 15.2.0）
- OpenCV 4.x（开发环境 4.10.0）

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

**必须在项目根目录运行**，因为程序使用相对路径读写文件。

### 视频模式（默认）

不带参数时，处理 `input/outpost.mp4`，结果写到 `output/`：

```bash
./build/armor_detect
```

显式指定输入输出：

```bash
./build/armor_detect input/outpost.mp4 output/result.mp4
```

### 图片模式

传入图片路径：

```bash
./build/armor_detect images/outpost_frame.png
```

## 输出

### 视频模式

| 文件 | 内容 |
| --- | --- |
| `output/result.mp4` | 标注视频：框、四角点、编号、距离 |
| `output/pose.csv` | 逐帧位姿数据 |
| `output/mask_red.png` | 第一帧的颜色 mask，调试用 |
| `output/bars.png` | 第一帧的灯条候选图，调试用 |

### 图片模式

| 文件 | 内容 |
| --- | --- |
| `output/mask_red.png` | 颜色分割 mask |
| `output/bars.png` | 灯条候选图 |
| `output/result.png` | 结果图：框、四角点、距离 |

### CSV 格式

```
frame_id,time_s,candidate_id,detect_valid,pnp_valid,
tx_m,ty_m,tz_m,distance_m,rx_rad,ry_rad,rz_rad,reprojection_error_px
```

| 字段 | 含义 |
| --- | --- |
| `frame_id` | 帧号，从 0 开始 |
| `time_s` | `frame_id / fps`，秒 |
| `candidate_id` | 同一帧内的候选编号，按中心点 x 升序、x 相同按 y 升序 |
| `detect_valid` | 是否检测到装甲板 |
| `pnp_valid` | solvePnP 是否成功 |
| `tx_m / ty_m / tz_m` | 装甲板中心在相机坐标系的位置，米 |
| `distance_m` | 到相机光心的直线距离，米 |
| `rx_rad / ry_rad / rz_rad` | 旋转向量，弧度 |
| `reprojection_error_px` | 四个角点的平均重投影误差，像素 |

没有候选的帧也会留一行，`detect_valid = 0`，位姿字段留空。

### 实测数据

完整视频（4210 帧）：

| 指标 | 结果 |
| --- | --- |
| 检测到装甲板的帧 | 2220 / 4210 |
| `distance` 范围 | 2.48 ~ 3.72 m，中位数 2.66 m |
| 重投影误差 | 0.05 ~ 2.80 px，中位数 0.98 px |
| `tz <= 0` | 0 行 |

## 方法说明

### 1. 颜色分割

选 **HSV + `inRange`**。三种方法实测对比：

| 方法 | 白像素数 | 问题 |
| --- | --- | --- |
| HSV `inRange(0,100,100)-(35,255,255)` | 1582 | 灯条完整，白板被 S 下限排除 |
| `R - B > 60` | 1707 | 多了墙面暖色反光 |
| 灰度 >120 且 `R-B>60` | 803 | 灯条变细、有断开 |

选 HSV 的理由：

1. 白色照明板 V 高但 S 接近 0，设 `Smin = 100` 一步排除
2. 色相 H 对光照变化不敏感
3. 红色跨色相边界在 HSV 里是显式的

参数来源：实测灯条区域得到 `H 3~31, S 122~223, V 134~255`，放宽为 `H 0~35, S 100~255, V 100~255`。

本图灯条 H 不跨色相边界，因此单段 `inRange` 即可，没有写第二段。

形态学：3×3 矩形核开运算，去掉小噪点。

### 2. 灯条筛选

| 量 | 计算 | 阈值 | 依据 |
| --- | --- | --- | --- |
| 面积 | `contourArea` | 60 ~ 400 | 噪点 ≤33，真灯条 172~193 |
| 长宽比 | 长边/短边 | ≥ 2.0 | 真灯条 3.4~4.6，方形噪点 1.0~1.8 |
| 填充率 | `contourArea/(w*h)` | ≥ 0.4 | 真灯条 0.69~0.77 |
| 倾角 | 长轴与竖直夹角 | ≤ 60° | 只排除接近水平的杂光 |

**倾角自己算，不用 `rect.angle`**：OpenCV 4.5 前后定义不同，且 0° 和 90° 都返回 90。改为从 `rect.points()` 取 4 个顶点，找最长边用 `atan2` 求方向，保留符号。

### 3. 灯条配对

| 条件 | 表达式 | 阈值 |
| --- | --- | --- |
| 颜色一致 | `A.color == B.color` | |
| 长度相近 | `min(len)/max(len)` | ≥ 0.7 |
| 方向协调 | `abs(A.angle - B.angle)` | ≤ 10° |
| 间距 | `dist / avgLen` | 1.0 ~ 3.0 |
| 垂直度 | 连线与灯条夹角偏离 90° | ≤ 15° |

间距用中心连线的**欧氏距离**，是旋转不变量。

垂直度检查：两根并排灯条的中心连线应垂直于灯条本身。`alongProj / dist` 是连线落在灯条方向上的比例，`asin` 后得到偏离角。

### 4. 四角点顺序

固定顺序：**左上 → 右上 → 右下 → 左下**。

用**象限法**排序：

1. 装甲板中心 `C` = 两根灯条中心的中点
2. 纵向轴 `v` = 两根灯条倾角的均值，换算成 `(sin, cos)`
3. 横向轴 `u` = `v` 顺时针转 90°，即 `(v.y, -v.x)`
4. 8 个顶点相对 `C` 求投影 `du`、`dv`，按正负分四象限
5. 每个象限里离 `C` 最远的点就是那个角

倾角均值满足交换律，两根灯条谁先谁后不影响结果，所以配对阶段不需要判断左右。

### 5. 位姿解算

**相机参数**（`src/pose.h` 中的 `CameraParams`，来源 `agent/参数.md`）：

| 参数 | 值 |
| --- | --- |
| `fx` | 1781.053454 |
| `fy` | 1781.686873 |
| `cx` | 510.5751561 |
| `cy` | 472.9735763 |
| `k1,k2,p1,p2,k3` | -0.04421, -0.10815, -0.00065, 0.000236, 0.46674 |
| 图像尺寸 | 1280 × 1024 |

**装甲板模型**（`src/pose.h` 中的常量）：

```
w = 0.135 m
h = 0.056 m
```

四个模型点（顺序必须与图像角点一致）：

```
0 左上 (-w/2, -h/2, 0)
1 右上 ( w/2, -h/2, 0)
2 右下 ( w/2,  h/2, 0)
3 左下 (-w/2,  h/2, 0)
```

模型原点在装甲板中心，z = 0 平面，单位米。

**调用方式**：

```cpp
cv::solvePnP(objectPoints, imagePoints, K, D, rvec, tvec,
             false, cv::SOLVEPNP_IPPE);
```

为什么用 `SOLVEPNP_IPPE`：

- 装甲板是**平面目标**（4 个点共面）
- 默认的 `SOLVEPNP_ITERATIVE` 对共面 4 点会抛异常（要求至少 6 点）
- IPPE 专门为平面目标设计，4 点即可，速度快

**输出含义**：

| 变量 | 含义 | 单位 |
| --- | --- | --- |
| `tvec` | 装甲板中心在相机坐标系的位置 | 米 |
| `rvec` | 旋转向量，方向为旋转轴、长度为旋转角 | 弧度 |
| `distance` | `sqrt(tx²+ty²+tz²)`，到相机光心的直线距离 | 米 |

相机坐标系：原点在光心，x 向右，y 向下，z 向前。

`tz` 是沿相机 z 轴的深度，**不等于**直线距离。

`rvec` 的三个分量**不是**横滚、俯仰、偏航三个欧拉角，不能逐个解释。

### 6. 重投影验证

```cpp
cv::projectPoints(objectPoints, rvec, tvec, K, D, reproj);
```

把模型点按估计位姿投回图像，与原始角点逐一比较：

```
e_i = sqrt((u_i - û_i)² + (v_i - v̂_i)²)
reprojection_error_px = (e_0 + e_1 + e_2 + e_3) / 4
```

实测中位数为 0.98 像素。

**重投影误差小不代表距离一定准。** `solvePnP` 只用 4 个点，可以通过调整位姿让投影对上，但绝对距离仍受相机参数和角点精度影响。所以还需要下面的独立核对。

### 7. 平面位姿二义性（已知限制）

平面目标有**两个合法的位姿解**：板子「往前俯」和「往后仰」投影出来的四边形几乎相同。

实测同一组角点：

| 解 | rvec | 旋转角 | 重投影误差 |
| --- | --- | --- | --- |
| 0 | (+0.580, -0.061, -0.079) | 33.7° | 0.24 px |
| 1 | (-0.401, +0.268, -0.012) | 27.6° | 0.26 px |

两解误差只差 0.02 像素。`cv::solvePnP` 内部会在两解间选择误差稍小的那个。

**后果**：相邻帧之间 `rvec` 可能正负跳变，`rvec` 数值不连续。

**不受影响**：两个解的 `tvec` 几乎相同（差 0.001 米以内），所以 `distance`、`tx`、`ty`、`tz` 都是稳定可信的。

**处理方式**：本阶段不做跟踪，任务书也不要求 `rvec` 连续。因此保持现状，在说明中注明：

> `rvec` 存在平面目标的固有二义性，相邻帧之间可能跳变；`distance` 和 `tvec` 稳定可信，`rvec` 仅作当前帧朝向的参考值。

如需固定选解，可改用 `cv::solvePnPGeneric` 取两个解，按「法线朝相机」或「与上一帧最接近」的规则筛选，但本阶段不做。

## 项目结构

```
OpenCV03/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── 问题记录.md
├── images/
│   └── outpost_frame.png     # 单张测试图
├── images_test/              # 旋转测试图（不提交）
├── input/
│   └── outpost.mp4           # 默认输入视频
├── output/                   # 运行时生成（不提交）
└── src/
    ├── armor.h               # 数据结构：Color / LightBar / Armor / Params
    ├── armor_detect.h        # 单帧检测接口
    ├── armor_detect.cpp      # 检测流程实现
    ├── pose.h                # 相机参数 / 位姿结果 / 装甲板尺寸
    ├── pose.cpp              # solvePose：solvePnP + 重投影
    └── main.cpp              # 入口：图片/视频两种模式、CSV 输出
```

## 接口

```cpp
// 单帧检测：一张 BGR 图 → 装甲板候选
bool detectArmor(const cv::Mat& bgr, const Params& p, DetectResult& out);

// 单块位姿：四个角点 → 位置、朝向、距离、误差
PoseResult solvePose(const cv::Point2f corners[4], const CameraParams& cam);
```

`detectArmor` 不依赖全局变量，不写死坐标。`solvePose` 只做数学，不依赖检测模块。

## 换场景需要做什么

**换相机或视频**：

1. 修改 `src/pose.h` 中 `CameraParams` 的 `K`、`D`、`width`、`height`
2. 确认视频分辨率与标定一致，否则位姿会算错

**换光照或环境**：

1. 在目标区域统计 H/S/V 的 10%~90% 分位
2. 填进 `src/armor.h` 的 `Params::hMin/hMax/sMin/sMax/vMin/vMax`
3. 跑一次看 `output/mask_red.png` 是否只保留目标

几何阈值是相对量，通常不需要改。

## 自测

**静态图片回归**：

```bash
./build/armor_detect images/outpost_frame.png
```

预期输出：

```
[装甲板 0] color=RED box=(654,287,86,40) center=(696.5,306.25)
  corners: 0=(654,294) 1=(739,287) 2=(739,318) 3=(654,326)
  barA=0 barB=1 valid=1
  tvec=(0.292515, -0.261993, 2.79939) m
  distance=2.82679 m  reproj=0.34158 px
```

**倾斜鲁棒性**（`images_test/` 下的三张旋转图）：

```bash
./build/armor_detect images_test/rot_15.png
./build/armor_detect images_test/rot_-20.png
./build/armor_detect images_test/rot_30.png
```

三张图都应输出 1 块装甲板。

**完整视频**：

```bash
./build/armor_detect input/outpost.mp4 output/result.mp4
```

完成后检查 `output/pose.csv` 的行数、`distance` 范围和重投影误差。

## 更新日志

### 2026-11-10

- 新增位姿解算模块 `src/pose.h` / `src/pose.cpp`
  - `CameraParams` 定义相机内参和畸变系数
  - `PoseResult` 保存位置、朝向、距离、重投影误差
  - `solvePose()` 调用 `solvePnP`（IPPE 方法）+ `projectPoints`
- `main.cpp` 接入位姿
  - 图片模式：终端打印 `tvec`、`distance`、`reproj`，结果图标注距离
  - 视频模式：逐帧解算，结果图标注距离，输出 `output/pose.csv`
- 新增 `drawPoseOverlay()` 绘制距离文字
- CSV 采用「内存收集 + 循环结束后一次性写入」，避免逐帧开关文件
- 记录平面位姿二义性问题（`rvec` 跳变，距离不受影响）

### 2026-11-09

- 重构分支，合并 `main` 的最新识别逻辑到视频分支
  - 配对改用欧氏距离 + 垂直度检查
  - 四角点改用倾角均值定轴，不再依赖左右判断
  - `Armor` 字段 `leftId/rightId` → `barAId/barBId`
- 视频编码改用 H.264（`avc1`），VS Code 可直接播放
- `.gitignore` 调整：`input/` 可提交，`output/` 忽略

### 2026-11-08

- 删除冗余的 `dyRatio` 判断（与 `gapRatio` + `angleFromPerp` 数学等价）
- README 同步最新识别逻辑

### 2026-11-04

- 首次接通视频输入输出
- 信息面板叠加到画面右下角
- 视频检测结果保存为 `output/result.mp4`

### 2026-11-03

- 完成静态图片装甲板检测
- 阶段：颜色分割 → 轮廓 → 灯条筛选 → 配对 → 四角点
- 输出 `mask_red.png`、`bars.png`、`result.png`
