# ============================================================
# 装甲板识别工程 —— CMake 配置
# 作用：告诉 CMake 用什么标准编译、去哪里找 OpenCV、要生成哪个程序
# ============================================================

# 要求 CMake 版本不低于 3.16（本机是 4.2.3，满足）
cmake_minimum_required(VERSION 3.16)

# 工程名叫 armor_detect，CXX 表示这是纯 C++ 工程（不涉及 C 编译器）
project(armor_detect CXX)

# 使用 C++17 标准。后面要用 std::filesystem（自动创建 output 目录），
# 它是 C++17 才进标准库的。不写这行，g++ 默认可能用旧标准导致编译失败
set(CMAKE_CXX_STANDARD 17)

# 如果编译器不支持 C++17 就直接报错，而不是悄悄降级成旧标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找系统安装的 OpenCV。
# 本机位置：/usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake
# 找到后 CMake 会自动定义这些变量：
#   OpenCV_LIBS          —— 需要链接的库列表（core、imgproc、imgcodecs 等）
#   OpenCV_INCLUDE_DIRS  —— 头文件搜索路径
# REQUIRED 表示找不到就报错停止，而不是继续往下走
find_package(OpenCV REQUIRED)

# 让 CMake 生成 build/compile_commands.json。
# 这个文件记录了每个源文件真实的编译命令（含 -I 头文件路径），
# VS Code 的 C/C++ 扩展读它之后才能正确找到 OpenCV 头文件，
# 否则会在 #include <opencv2/opencv.hpp> 下面画红波浪线（其实能编译）。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 定义一个可执行程序：
#   名字叫 armor_detect（生成在 build/armor_detect）
#   由 src/main.cpp 编译得到
# 必须先有 target（这一行），后面的 target_* 命令才有作用对象
add_executable(armor_detect
    src/main.cpp
    src/armor_detect.cpp
    src/pose.cpp
)

# 让编译器能找到 <opencv2/opencv.hpp> 这类头文件。
# 必须写在 add_executable 之后：target_include_directories 是给
# 「已存在的目标」加属性的，目标不存在会报
# "Cannot specify include directories for target ... which is not built by this project"
target_include_directories(armor_detect PRIVATE ${OpenCV_INCLUDE_DIRS})

# 把 OpenCV 的库链接到 armor_detect 上。
# 只 #include 头文件不够，链接阶段还需要库文件，否则报 undefined reference
target_link_libraries(armor_detect PRIVATE ${OpenCV_LIBS})
