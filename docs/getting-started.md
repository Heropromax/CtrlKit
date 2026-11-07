# 入门指南

## 在自己的项目中使用

CtrlKit 是头文件库。将仓库的 `include/` 目录加入头文件搜索路径，启用 C++20，无需链接额外的 CtrlKit 二进制库。

| 头文件 | 用途 |
|---|---|
| `<ctrlkit/ctrlkit.hpp>` | Vec、类型萃取、基础运算、限幅、角度工具 |
| `<ctrlkit/math.hpp>` | 向量初等函数和幂运算 |
| `<ctrlkit/quat.hpp>` | 四元数和姿态转换 |
| `<ctrlkit/dynamic.hpp>` | 低通滤波器和积分器 |

后三个头文件会包含基础头文件；`ctrlkit.hpp` 本身不是总入口。命名空间为 `ctrlkit`，简写为 `ctk`。

### 完整示例

保存以下内容为 `example.cpp`：

```cpp
#include <ctrlkit/dynamic.hpp>

int main()
{
    ctk::Lpf1st<ctk::Vec3f> filter(0.001f, 0.02f);
    ctk::Integrator<ctk::Vec3f> integral(0.001f);
    const ctk::Vec3f input{1.0f, 2.0f, 3.0f};
    const auto filtered = filter.step(input);
    const auto accumulated = integral.step(filtered);
    return accumulated[0] > 0.0f ? 0 : 1;
}
```

在仓库根目录使用 GCC 编译：

```sh
g++ -std=c++20 -Iinclude example.cpp -o example
```

在 Visual Studio 开发者终端中使用 MSVC：

```powershell
cl /std:c++20 /EHsc /utf-8 /I include example.cpp /Fe:example.exe
```

`step()` 每次推进一步。实际程序应按传入的采样周期调用；模块不会读取时钟或自行调度。需要恢复状态时调用 `reset(value)`。

## 工具链要求

项目设定的工具链基线为 GCC 12+、Clang 14+、MSVC VS2022。这是目标支持范围，不代表所有版本和目标芯片均已验证。

库本体需要 C++20 concepts 和 `<numbers>`。涉及 `<cmath>` 的编译期求值能力取决于编译器与标准库，标记 `constexpr` 不代表所有调用都能在编译期执行。

仓库测试程序还使用 C++23 `<print>`，需要实际提供该头文件和实现的标准库版本。满足库基线的工具链不一定能直接构建仓库测试程序。

嵌入式接入方式相同：将 `include/` 加入现有固件工程并启用 C++20；外设驱动、时钟、任务和中断由应用负责。更多边界见[设计说明](design.md)。

## 构建本仓库并运行测试

仓库使用 CMake，当前目标为 `MainApp` 和模板占位目标 `test_empty`。有效的 CtrlKit 接口验证集中在 `src/main.cpp`。

在已配置编译器的终端中执行：

```sh
cmake -S . -B build
cmake --build build --config Debug
```

Visual Studio 等多配置生成器通常运行：

```powershell
.\build\Debug\MainApp.exe
```

单配置生成器通常运行 `./build/MainApp`；Windows 下为 `build/MainApp.exe`。已有构建缓存时，应沿用原生成器与工具链。

`MainApp` 打印通过/失败计数，失败返回非零退出码。当前 CMake 没有注册 CTest 测试，应直接运行该程序。

## Windows / VS Code 开发环境

安装带 C++ 工具集的 Visual Studio 或 Build Tools、CMake，以及 VS Code 的 CMake Tools 扩展。建议使用支持 C++23 `<print>` 的 MSVC 工具集。

在 VS Code 命令面板中：

1. `CMake: Select a Kit`：选择编译器与平台。
2. `CMake: Select a Variant`：选择 Debug 或 Release。
3. `CMake: Set Launch/Debug Target`：选择 `MainApp`。
4. 构建并运行或调试。

修改 `CMakeLists.txt` 后保存并重新配置。构建失败先检查诊断和工具链；如缓存与生成器冲突，可以使用新的构建目录重新配置。

修改库之前请阅读[贡献指南](../CONTRIBUTING.md)。接口参数与使用约束见 [API 文档](api.md)。
