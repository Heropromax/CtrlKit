cmake_minimum_required(VERSION 3.10)

project(CppApp)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

## 👉 选择 cmake使用的编译器和平台 Ctrl + Shift + P -> CMake: Select a Kit
## 👉 选择 release 或 debug 模式：Ctrl + Shift + P -> CMake: Select a Variant
## 👉 选择要测试/Debug的程序: Ctrl + Shift + P -> CMake: Set Launch/Debug Target

# 告诉编译器使用 UTF-8 编码
if (MSVC)
    add_compile_options(/source-charset:utf-8)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-finput-charset=UTF-8 -fexec-charset=UTF-8)
endif()


# # 添加 OpenCV 库
# find_package(OpenCV REQUIRED)


include_directories(
    ${CMAKE_SOURCE_DIR}/include # 包含项目的头文件目录

    # ${OpenCV_INCLUDE_DIRS} # OpenCV 的头文件目录
)

# 收集源文件：引入 CONFIGURE_DEPENDS 确保新建文件时构建系统能自动重新扫描
file(GLOB_RECURSE MODULE_SRC_FILES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/*.cpp")
# 健壮的路径排除：使用 ABSOLUTE 确保在所有平台上路径字符串格式一致
get_filename_component(MAIN_APP_PATH "${CMAKE_SOURCE_DIR}/src/main.cpp" ABSOLUTE)
# 执行排除操作
list(REMOVE_ITEM MODULE_SRC_FILES "${MAIN_APP_PATH}")


################################## 主程序 ####################################
# 添加主程序，可执行文件，入口为 main.cpp
add_executable(MainApp "${MAIN_APP_PATH}" ${MODULE_SRC_FILES})
# target_link_libraries(MainApp PRIVATE ${OpenCV_LIBS})

################################## 测试程序 ####################################
# 添加测试程序
add_executable(test_empty tests/test_empty.cpp )
# target_link_libraries(test_empty PRIVATE ${OpenCV_LIBS})

