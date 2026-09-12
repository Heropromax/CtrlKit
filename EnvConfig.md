# VSCode C++ 项目模板 (Windows)
## 系统要求
MSVC 推荐安装Visual Studio

## 程序编译Debug设置
- 选择 cmake使用的编译器和平台 Ctrl + Shift + P -> CMake: Select a Kit

- 选择 release 或 debug 模式：Ctrl + Shift + P -> CMake: Select a Variant

- 选择要测试/Debug的程序: Ctrl + Shift + P -> CMake: Set Launch/Debug Target

## 其他
- 有的时候编译不过可以删除build文件

- 修改`CMakeLists.txt`或新增代码文件时确保 `CTRL+S` 保存刷新CMake缓存
