# CtrlKit 设计说明

> **一句话定位**：一个**极简、头文件式、专注实时控制**的 C++ (>=C++20) 数学与动态模块库。
> 只提供"积木"，不替用户搭系统。
>
> 编译/环境配置请看 `README.md`，本文只讲**设计意图、现状与路线图**。

---

## 0. 快速上手（TL;DR）

```cpp
#include <ctrlkit/ctrlkit.hpp>   // Vec / scalar_t / 常用数学
#include <ctrlkit/math.hpp>      // 逐元素 sin/cos/exp/log ...
#include <ctrlkit/quat.hpp>      // 四元数姿态
#include <ctrlkit/dynamic.hpp>   // Lpf1st / Lpf2nd / Integrator

using namespace ctk;             // ctk == ctrlkit

// —— 向量 ——
Vec3f a{1, 2, 3};
Vec3f b{4, 5, 6};
Vec<float,3> c{4, 5, 6};
Vec3f c = (a + b) * 2.0f - b / a;     // 逐分量 + 标量广播，任意组合
float d = dot(a, b);                   // 自由函数，不是成员函数
float n = norm(c);

// —— 四元数（旋转）——
Quat<float> q = EulerZYX2Quat(Vec3f{0.3f, -0.2f, 0.1f});
Vec3f r = rotvec(q, Vec3f{1, 0, 0});   // 旋转一个向量
Vec3f e = Quat2EulerZYX(q);

// —— 动态模块（统一 step / reset 接口）——
Lpf1st<double> lpf(0.001, 0.02);        // (dt, tc)
Integrator<double> integ(0.001);        // (dt)
double y = lpf.step(imu_raw);
double s = integ.step(y);
```

**统一约定**：所有动态模块都是 `step(input) -> output` + `reset(initial)`，都带一个可选的"初始值"参数以避免启动瞬态。

---

## 1. 设计哲学

### 1.1 定位与边界

- 这是**实时控制**用的基础库，不是"数值全家桶"。
  凡是**实际控制中几乎用不到**的东西（高阶滤波器、定点数、运动规划……），优先级一律压低或不做。
- 库的职责是**提供积木**；**怎么组合由用户决定**。
- 头文件式（header-only），零构建依赖，`constexpr` 全覆盖。

### 1.2 极简：只把"最必要的数据和方法"放进 `struct`

`Vec` / `Quat` 里**只放**：

- **数据成员**（`Vec::data`、`Quat::w/x/y/z`）
- **运算符**（`+ - * /`、复合赋值、一元、比较）
- **下标访问** `operator[]`

**不放**：`norm`、`dot`、`cross`、`sum`、`normalize`、`sin`……全部是**自由函数**。

**为什么？**

| 理由 | 说明 |
|---|---|
| **避免类接口膨胀** | `Vec` 成员的职责边界清晰=运算符+数据；加 20 个数学函数会变成"上帝类" |
| **降低耦合** | 自由函数只依赖 `Vec` 的**公开数据/运算符**，不依赖内部实现，实现可自由改 |
| **可扩展** | 新增能力（`sin`、未来的 `clamp`）不需要改 `Vec` 的定义 |
| **可读性** | `dot(a, b)` / `norm(v)` 比 `a.dot(b)` 更接近数学记法 |
| **ADL 可用** | 自由函数在 `ctrlkit` 命名空间里，`sin(v)` 直接就能找到，无需写 `ctk::` |

> **判断标准**：如果是"运算符"或"访问数据的方式" → 放进 `struct`；
> 如果是"对信号做的运算" → 写成自由函数。

### 1.3 显示优于隐式（Explicit over implicit）

这是本项目反复贯彻的一条原则：

- **没有**隐式的 `scalar → Vec` 转换构造函数（历史上删掉过）。
  理由：会造成重载歧义，且让"标量被悄悄提升成向量"这类 bug 难以察觉。
- **没有**四元数的 `operator*=(Quat)`。
  理由：四元数乘法**不满足交换律**，左乘/右乘对应不同坐标系语义（本体系 vs 世界系）。
  与其把它藏起来，不如**强制用户显式书写** `q = q * r` 或 `q = r * q`。
  （已在源码注释中写明"为什么不提供"，防止后人"顺手补回来"。）
- **元素类型转换是 `explicit`** 的。
  `static_cast<Vec<double,3>>(v)` 可以，隐式转换不行 —— 否则混合类型的表达式会出现二义性。
- **除法保留精确语义**，不预先做"倒数乘法"优化。
  是否牺牲精度换速度，由用户通过 `/fp:fast`、`-ffast-math` 决定，**库不替用户决定**。

### 1.4 一个类型参数 + 标量萃取 → 同一套代码同时支持标量与多维信号

这是本项目**最核心的设计模式**。

```cpp
template <typename T>          // T = 信号类型：float / double / Vec<...>
class Lpf1st
{
    using Scalar = scalar_t<T>;   // 系数类型：永远退化为标量
    Scalar alpha;                 // ✅ 系数是标量 → 所有轴共用一个 α
    T prev_output;                // ✅ 状态是信号类型
};
```

`scalar_t<T>` 是一个 trait：

| 输入 | 输出 |
|---|---|
| `float` / `double` | 它自己 |
| `Vec<float, 3>` | `float` |

**收益**：`Lpf1st<double>` 和 `Lpf1st<Vec3f>` 用的是**同一份代码**，无需特化，也无需写两遍。

**这是新增动态模块时的标准写法**，请照抄这个模式：

```cpp
template <typename T>
class MyFilter
{
public:
    using Scalar = scalar_t<T>;
    // 系数用 Scalar，状态与输入输出用 T
private:
    Scalar coef;
    T state{};
};
```

### 1.5 聚合类型优先（Aggregate-first）

`Vec` 和 `Quat` 都是**聚合类型**（没有用户声明的构造函数）：

```cpp
Vec3f v{1, 2, 3};      // 聚合初始化，逐元素
Vec3f z;               // 全零（靠 NSDMI：T data[N] = {})
Quat<float> q;         // 单位四元数（w=1）
```

**代价与取舍**：一旦加了构造函数，聚合性就没了，`Vec v{1,2,3}` 也就废了。
所以连"元素类型转换"都特意用**转换运算符**而不是转换构造函数：

```cpp
template <typename U>
constexpr explicit operator Vec<U, N>() const;   // 不是构造函数 → 聚合性保留
```

### 1.6 数值稳定优先

- 二阶滤波器用 **TDF-II（转置直接 II 型）**，而不是直接 I 型 —— 状态变量动态范围小，定点/浮点都更稳。
- 双线性变换带 **频率预畸变（pre-warping）**，保证 `fc` 处增益**恰好** −3dB。
- 滤波器支持**对齐直流稳态的初值**，启动时**无冲击**（`Lpf2nd` 的 `reset` 公式是解析推导出来的）。
- `norm` / `normalize` 对零向量有兜底（返回单位四元数），不产生 NaN。

### 1.7 `constexpr` 全覆盖

能标 `constexpr` 就标。常量表达式可折叠 → 零运行期开销。

> ⚠️ **已知限制**：MSVC 的 `std::sin/cos/tan/log/atan2` 等**不是 `constexpr`**，
> 所以涉及 `<cmath>` 的函数（`math.hpp`、`quat.hpp` 的欧拉角转换、`Lpf2nd` 的构造函数）
> 在 **MSVC 上无法真正编译期求值**，只能运行期调用。
>
> **仍然保留 `constexpr`** —— 在 GCC/Clang 等支持 constexpr `<cmath>` 的工具链上它就能用，
> 属于"免费的前向兼容"；在 MSVC 上自动降级为普通函数，无副作用。

### 1.8 不替用户做决定

明确**不提供**（详见 §3.4）：PID、轨迹规划、统一采样周期/调度。

**理由**：这些东西形式太多（PID 有无穷多种变体、轨迹有梯形/S曲线/样条……），
库一旦"替你选了"，用户就被绑架。用户拿基础积木自己组合，反而更快、更透明。

### 1.9 命名与风格约定

| 类别 | 风格 | 例子 |
|---|---|---|
| 类型 / 类 | `PascalCase` | `Vec`, `Quat`, `Lpf1st`, `Lpf2nd`, `Integrator` |
| 函数 / 变量 / 别名 | `snake_case` | `norm_sq`, `scalar_t`, `deg2rad`, `prev_output` |
| 常量变量模板 | `snake_case` | `pi<T>`, `deg2rad<T>`, `rad2deg<T>` |
| 命名空间 | `ctrlkit`，别名 `ctk` | `ctk::Vec3f` |

- 动态模块统一的接口名：**`step()`** + **`reset()`**。
- 类型别名后缀表示元素类型：`Vec3f` = `Vec<float,3>`，`Vec3d` = `Vec<double,3>`。

### 1.10 头文件组织：谁用谁包含

每个头文件都**显式包含自己用到的标准库**（`<cmath>`、`<numbers>`、`<type_traits>`），
**不依赖传递包含**。这样换包含顺序、单独引用某个头文件都不会出问题。

依赖方向：

```
ctrlkit.hpp   ← 基础（Vec / scalar_t / 常量）
   ↑
   ├── math.hpp      逐元素数学函数
   ├── quat.hpp      四元数（依赖 Vec）
   └── dynamic.hpp   动态模块（依赖 scalar_t）
```

### 1.11 语法糖只对外，内部只用标准库

`pi<T>` / `deg2rad<T>` / `rad2deg<T>` / `Vec3f` / `Vec3d` 定位是**给用户的短名字**。

**库内部实现一律直接用 `std::numbers::pi_v<T>`**，不依赖这些糖。
好处：将来若删掉这些别名，**库自身一行都不会坏**（只有测试会需要跟着改）。

### 1.12 测试即验证：模板必须被实例化

`src/main.cpp` 是冒烟测试（`static_assert` 编译期 + `check()` 运行期，失败返回非零退出码）。

> **重要教训**：模板**只有被实例化时才会被完整检查**。
> 这些头文件曾经没有任何调用者，导致 `cross()` 里一个 MSVC 不支持写法
> （括号聚合初始化）潜伏了很久，直到加了测试才暴露。
>
> **所以：新增/修改模板代码后，务必在 `src/main.cpp` 里实例化一遍并跑通。**

---

## 2. 已完成功能（现状）

### 2.1 文件结构

```
include/ctrlkit/
  ctrlkit.hpp    基础：Vec、scalar_t、常量
  math.hpp       逐元素初等函数
  quat.hpp       四元数姿态
  dynamic.hpp    动态/离散模块
src/main.cpp     冒烟测试（覆盖全部对外接口）
```

工程：**C++23 + CMake + MSVC**，目标 `MainApp`；测试直接跑 `MainApp`。

### 2.2 `ctrlkit.hpp`

**`Vec<T, N>`**（聚合类型，`static_assert(N >= 2)`）

| 能力 | 说明 |
|---|---|
| `data[N]` + `operator[]` | 像普通数组一样读写 |
| `Vec ⊗ Vec` 的 `+ - * /` | **逐分量**（Hadamard；`*` **不是**点积） |
| `Vec ⊗ 标量` 的 `+ - * /` | 广播 |
| `标量 ⊗ Vec` 的 `+ - * /` | 友元，支持 `2.0f * v` 这种写法 |
| 一元 `+ -` | |
| `+= -= *= /=` | 向量版 + 标量版 |
| `==`（`!=` 自动生成） | 逐分量精确比较 |
| `explicit operator Vec<U, N>()` | `static_cast<Vec<double,3>>(vf)`，逐元素 `static_cast`（会截断） |

**标量萃取**：`scalar_of<T>` / `scalar_of<Vec<U,N>>` / `scalar_t<T>`（标准库同款 trait 结构）

**自由函数**：`sum`、`dot`、`norm_sq`、`norm`、`cross`（仅三维）

**语法糖**：`Vec3f`、`Vec3d`、`pi<T>`、`deg2rad<T>`、`rad2deg<T>`

**命名空间别名**：`namespace ctk = ctrlkit;`

### 2.3 `math.hpp`

**逐元素**（仅向量重载）：
`sin`、`cos`、`tan`、`asin`、`acos`、`atan`、`exp`、`log`

> 与 `<cmath>` 同名，但参数是 `Vec<T,N>`，因此**不会干扰**标量调用。
> ADL 让 `sin(v)` 无需写命名空间 —— 这是有意为之。

### 2.4 `quat.hpp`

**`Quat<T>`**（聚合类型，仅浮点；默认单位四元数 `w=1`）

| 能力 | 说明 |
|---|---|
| `operator*(Quat)` | Hamilton 积（复合旋转） |
| `operator*(T)` / `operator/(T)` | 标量乘除 |
| `标量 * Quat` / `标量 / Quat` | 友元 |
| `*=(T)` / `/=(T)` | 标量复合赋值 |
| `conj()` | 共轭（单位四元数下即逆） |

**自由函数**：

| 函数 | 说明 |
|---|---|
| `norm_sq` / `norm` | 模长平方 / 模长 |
| `normalize` | 归一化（零模长时返回单位四元数） |
| `rotvec(q, v)` | 旋转三维向量（**仅单位四元数**；用两次叉乘的高效公式） |
| `Quat2EulerZYX` / `EulerZYX2Quat` | 四元数 ↔ 欧拉角（ZYX，弧度） |
| `Quat2EulerZYXDeg` / `EulerZYXDeg2Quat` | 同上，度数 |

> 欧拉角向量顺序统一为 `[yaw, pitch, roll]`（下标 0/1/2），两个方向**互相一致**。

### 2.5 `dynamic.hpp`

所有模块统一：`step(input) -> output`、`reset(value)`、可选初始值、支持标量与 `Vec`。

| 模块 | 构造参数 | 说明 |
|---|---|---|
| **`Lpf1st<T>`** | `(dt, tc, initial_output=0)` | 一阶低通 / 指数平滑，`α = dt/(tc+dt)` |
| **`Lpf2nd<T>`** | `(dt, fc, zeta=0.7071, initial_output=0)` | 二阶低通：双线性变换 + 预畸变 + **TDF-II**，直流增益恒为 1 |
| **`Integrator<T>`** | `(dt, initial_state=0)` | 前向欧拉（左矩形）积分 |

**亮点**：`Lpf2nd` 的 `reset()` 会把状态直接置到**直流稳态**（解析推导），
所以给定 `initial_output` 后**没有启动冲击**。

### 2.6 测试

`src/main.cpp` —— 覆盖 `Vec`、`Vec` 元素类型转换、`Quat`、`Lpf1st`、`Lpf2nd`、`Integrator`、`math.hpp` 的全部对外接口。

- 编译期：`static_assert`（含 `Vec` 运算、四元数乘法、`scalar_t` 萃取、滤波器常量）
- 运行期：数值验证（如 **`fc` 处恰好 −3dB**、**阶跃超调 ≈ 4.32%**、欧拉角往返、滤波器收敛）

> 运行 `build/Debug/MainApp.exe`，末尾打印 `passed / failed`，失败返回非零退出码。

---

## 3. 路线图（Roadmap）

### 3.1 🔴 高优先级

| 项 | 说明 |
|---|---|
| **`Saturation`（限幅）** | 执行器指令保护，最基础也最必需 |
| **`IntegratorLimit`** | 带**输出限幅**的积分器（anti-windup 地基） |
| **基础数学工具** | `abs`、`sign`、`clamp`、`min`、`max`、`sqrt`、`pow`（先这些，不够再添） |
| **微分模块（Derivative）** | 见下方说明 —— PID 只差这一块 |

> **关于 PID**：**本库不提供 PID**。
> 因为在真实工作流里 PID 不是一个"模块"，而是**三件东西相加**：
> 比例（就是乘法）、**积分（`Integrator` 已有）**、微分（缺）。
> 所以只要补上**微分模块**，用户自己组合即可 —— 而 PID 的变体无穷多，
> 库不应该替用户选定某一种形式（微分可以预计算、可以外部提供、可以带滤波……）。

### 3.2 🟡 中优先级

| 项 | 说明 |
|---|---|
| **健壮性保护** | `dt <= 0`、除零、`fc` 超 Nyquist（`tan` 变负）、NaN 传播等的校验/兜底。**下次再加**。 |

### 3.3 🟢 低优先级

| 项 | 说明 |
|---|---|
| **轴角 ↔ 四元数** | `from_axis_angle` / `to_axis_angle`。**几乎用不到，优先级放低**。 |
| **高阶/多种滤波器** | 任意阶 Butterworth、带通/带阻等。**实时控制几乎用不到高阶滤波**，优先级很低。 |
| **定点数（fixed-point）支持** | 比较麻烦，**暂不做**。 |

### 3.4 ⛔ 明确**不做**（设计决策，非"待办"）

| 不做 | 理由 |
|---|---|
| **PID 控制器** | 用户自己组合 P + I + D（见 §3.1） |
| **轨迹 / 运动规划** | **用户自己写**，库不替用户做决定 |
| **统一的采样周期 / 调度** | **由用户决定**，库只提供基础模块 |
| **传输延迟 / Delay 模块** | 暂无需求 |
| **隐式类型转换** | 见 §1.3，明确拒绝 |

---

## 4. 给 AI 助手 / 新加入者的硬性约定

改这个库时，请遵守：

1. **不要加隐式转换**（构造函数、非 `explicit` 的转换运算符）。
2. **不要给 `Vec` / `Quat` 加构造函数** —— 会破坏聚合初始化。
3. **不要把数学函数塞进 `struct` 当成员** —— 写成 `ctrlkit` 命名空间下的自由函数。
4. **新增动态模块必须用 `scalar_t<T>` 模式**，让标量和 `Vec` 共用一套代码；
   接口用 `step()` / `reset()`，并提供可选初始值。
5. **不要替用户做设计决定**（PID、轨迹、调度、精度策略……）。
6. **自己用到的标准库头文件自己 `#include`**，不要靠传递包含。
7. **改完模板代码，务必在 `src/main.cpp` 里实例化并跑通全部测试**——
   模板不被实例化就不会被检查。
8. 命名遵守 §1.9 的风格。
9. **不要修改 `README.md`**（那是编译/环境指南）。
