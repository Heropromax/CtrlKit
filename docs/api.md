# API 文档

本文描述当前实现的公开接口。设计理由见[设计说明](design.md)，接入步骤见[入门指南](getting-started.md)。命名空间为 `ctrlkit`，别名为 `ctk`。

## 基础类型：ctrlkit.hpp

### `Vec<T, N>`

固定维度聚合类型，要求 `N >= 2`。数据为 `T data[N] = {}`，默认各分量为零。

```cpp
ctk::Vec3f a{1.0f, 2.0f, 3.0f};
ctk::Vec<float, 3> b{4.0f, 5.0f, 6.0f};
auto c = (a + b) * 2.0f;
auto d = ctk::dot(a, b);
auto wide = static_cast<ctk::Vec<double, 3>>(a);
```

| 接口 | 行为 |
|---|---|
| `operator[](i)` | 返回分量引用，不检查越界；调用者保证 `i < N` |
| 向量间 `+ - * /` | 同类型、同维度逐元素运算；`*` 不是点积 |
| 向量与标量 `+ - * /` | 左右两侧均支持标量广播，标量参数类型为 T |
| `+= -= *= /=` | 支持向量或标量右操作数 |
| 一元 `+ -` | 逐元素正负号 |
| `==`、`!=` | 逐元素精确比较，不做浮点容差比较 |
| `explicit operator Vec<U, N>()` | 逐元素 static_cast，可能损失精度或截断 |

不提供标量到向量的隐式转换或构造函数。除法要求分母有效；整数运算遵循 C++ 整数规则。

快捷别名：`Vec2f`、`Vec3f`、`Vec4f`，以及对应的 `Vec2d`、`Vec3d`、`Vec4d`。

### 基础自由函数

| 接口 | 行为 |
|---|---|
| `sum(v)` | 分量求和 |
| `dot(a, b)` | 点积 |
| `norm_sq(v)` | 模长平方 |
| `norm(v)` | 模长，即平方和的平方根；零向量返回零 |
| `cross(a, b)` | 三维叉积，仅接受 `Vec<T, 3>` |
| `saturate(value, min, max)` | 标量限幅，返回值位于闭区间内 |
| `saturate(v, min, max)` | 向量逐分量限幅，所有轴共用标量上下限 |

`saturate` 使用 `std::clamp`；调用者必须保证 `min <= max`，不应依赖它修复 NaN。当前没有向量 `normalize` 接口。

### 标量萃取与角度工具

| 接口 | 行为 |
|---|---|
| `scalar_of<T>::type`、`scalar_t<T>` | 对标量返回自身类型；对 `Vec<U, N>` 返回 U |
| `pi<T>` | 浮点类型的 π 常量 |
| `pi_f`、`pi_d` | float / double 的 π 常量 |
| `deg2rad(x)` | 度转弧度，支持浮点标量和浮点向量 |
| `rad2deg(x)` | 弧度转度，支持浮点标量和浮点向量 |

## 逐元素数学：math.hpp

以下重载接受 `Vec<T, N>`，返回同类型向量：

`sin`、`cos`、`tan`、`asin`、`acos`、`atan`、`exp`、`log`。

三角函数使用弧度。函数逐元素调用对应标准库数学函数，不额外检查定义域；调用者负责保证输入有效。

| 接口 | 行为 |
|---|---|
| `pow(v, exponent)` | 指数为元素类型 T 的标量，各分量调用 std::pow |
| `pow<E>(v)` | 编译期整数指数，使用乘法展开；零指数返回全 1 向量，负指数取倒数 |

负指数要求分量非零。整数指数模板适合小指数；当前为线性展开，避免使用极大指数或 int 最小值。此头文件不提供对应的标量数学重载，标量通常使用 std 函数。

## 姿态：quat.hpp

### `Quat<T>`

仅支持浮点元素类型的聚合类型，分量顺序为 `w, x, y, z`；默认值 `{1, 0, 0, 0}` 表示单位四元数。别名为 `Quatf`、`Quatd`。

| 接口 | 行为 |
|---|---|
| `q * r` | Hamilton 积；顺序有意义 |
| `q * s`、`s * q`、`q / s` | 分量与标量相乘或相除 |
| `s / q` | 标量分别除以四个分量；不是乘以四元数逆 |
| `q *= s`、`q /= s` | 标量复合赋值；不提供四元数复合乘法 |
| `q.conj()` | 共轭；仅单位四元数的共轭等于其逆 |
| `norm_sq(q)`、`norm(q)` | 模长平方和模长 |
| `normalize(q)` | 模长小于 1e-2 时返回单位四元数，否则归一化 |

`normalize` 不负责修复任意 NaN 或无穷值。标量除法与逐分量除法均要求分母有效。

### 旋转与欧拉角

| 接口 | 行为与前置条件 |
|---|---|
| `quat_rotate(q, v)` | 旋转三维向量；要求单位四元数，不会自动归一化 |
| `euler2quat_zyx(euler)` | ZYX 欧拉角转四元数，向量顺序 [yaw, pitch, roll]，单位弧度 |
| `quat2euler_zyx(q)` | 单位四元数转上述欧拉角；俯仰反三角输入越界时作限幅处理 |

旋转公式对应主动旋转 `q * (0, v) * conj(q)`。在这一约定下，`q * r` 对向量的作用顺序是先 r 后 q。机体系、世界系的具体映射由应用明确。

欧拉角存在奇异点和表示多解，不应期待任意欧拉角往返后数值逐项相等。度数通过角度工具组合：

```cpp
auto q = ctk::euler2quat_zyx(ctk::deg2rad(ctk::Vec3f{30.0f, 0.0f, 0.0f}));
auto degrees = ctk::rad2deg(ctk::quat2euler_zyx(q));
```

## 动态模块：dynamic.hpp

模板参数 T 为信号类型，常用浮点标量或浮点 Vec。`Scalar = scalar_t<T>` 为系数类型；向量所有轴共用系数，各轴状态独立。

所有模块提供 `step(const T &input) -> T` 和 `reset(const T &value = T{})`。step 更新内部状态后返回本步输出；模块不读取时钟、不提供调度。dt 单位为秒，每次调用对应一个采样周期。

### 构造与算法

| 类型 | 构造参数顺序 | 行为 |
|---|---|---|
| `Lpf1st<T>` | `(dt, tc, initial_output = T{})` | alpha = dt / (tc + dt)，y = alpha*x + (1-alpha)*y_prev，tc 单位秒 |
| `Lpf2nd<T>` | `(dt, fc, zeta = Scalar(0.7071), initial_output = T{})` | 双线性变换、频率预畸变、TDF-II；fc 单位 Hz，zeta 为无量纲阻尼比 |
| `Integrator<T>` | `(dt, initial_state = T{})` | state += input * dt，返回更新后的状态 |
| `IntegratorSat<T>` | `(dt, initial_state = T{}, sat_min = lowest(), sat_max = max())` | 同上，但每步将状态夹到上下限内 |

IntegratorSat 默认上下限来自 `std::numeric_limits<Scalar>`，是有限数值边界。上下限对向量所有分量共用。

### 参数前置条件

- 使用有限浮点参数，dt > 0。
- 一阶低通要求 tc >= 0；tc = 0 时直接跟随输入。
- 二阶低通要求 0 < fc < 1 / (2*dt)，并使用正阻尼比。
- 带限幅积分器要求 sat_min <= sat_max。

这些条件目前主要由调用者保证，构造函数没有完整的运行期校验。限幅也不能避免限幅前的溢出或 NaN 传播。

二阶低通预畸变将设计频率映射到 fc；**仅在 Butterworth 阻尼比约为 1/sqrt(2) 时，该频率处增益约为 −3 dB**，不适用于任意 zeta。

### 初值与 reset

| 模块 | 行为 |
|---|---|
| Lpf1st | 构造与 reset 设置上一次输出 |
| Lpf2nd | 按恒定输入、输出均等于给定值的直流稳态设置内部状态 |
| Integrator | 构造与 reset 设置积分状态 |
| IntegratorSat | 构造直接保存初值，首次 step 后限幅；reset 立即对给定值限幅 |

滤波器对齐初始输出，有助于消除与该稳态一致的输入下的启动瞬态，不代表对任意输入变化都没有瞬态。

除 Lpf2nd 构造函数外，上述模块的构造和 step/reset 标有 constexpr；实际常量求值还取决于所调用操作和工具链。
