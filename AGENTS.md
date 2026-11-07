# 项目工作入口

CtrlKit 是面向实时控制的 C++20 头文件库。人和编程助手的详细开发规范统一维护在 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 开始工作

- 修改库：先读贡献指南，必要时阅读 [design](docs/design.md)。
- 查接口：阅读 [API](docs/api.md)，与实现和测试对照。
- 构建运行：阅读 [getting-started](docs/getting-started.md)。
- 查计划与缺口：阅读 [roadmap](docs/roadmap.md)，不要将计划当成已实现功能。

## 代码位置

库实现在 include/ctrlkit/ 的四个头文件中。ctrlkit.hpp 是基础头文件，不是总入口。src/main.cpp 是主要测试入口，tests/test_empty.cpp 是占位程序。

## 关键约束

- 保持 Vec / Quat 的聚合性和显式转换；新增数学计算优先用自由函数。
- 动态模块使用 T 信号、`scalar_t<T>` 系数，以及 step/reset 接口。
- 核心保持固定尺寸存储和有界工作量，不引入堆分配、异常、RTTI 或虚函数。
- 直接包含所需标准库头文件，不依赖传递包含。
- 修改模板行为后实际实例化并运行相关验证；仓库测试需要支持 C++23 `<print>` 的工具链。

## 检查与文档

常用命令为 `cmake -S . -B build`、`cmake --build build --config Debug`，然后运行 MainApp（多配置 Windows 通常为 build/Debug/MainApp.exe）。沿用已有生成器，当前未注册 CTest。

接口事实更新 API，设计变更更新 design，计划更新 roadmap。影响项目概览时才更新 README。详细规范以贡献指南为准；当前实现偏差见路线图，不要把设计目标当成已有保证。
