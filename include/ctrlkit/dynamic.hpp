#pragma once

#include <cmath>
#include <numbers>

#include <ctrlkit/ctrlkit.hpp>

namespace ctrlkit
{
    // 一阶低通滤波器（一阶指数平滑）
    // T：信号类型，可以是标量（float / double）或 Vec<...>。
    // 系数（dt、tc、alpha）统一使用 scalar_t<T>：所有轴共用同一个 alpha。
    template <typename T>
    class Lpf1st
    {
    public:
        using Scalar = scalar_t<T>;

        // 构造函数，传入采样周期 dt 和时间常数 tc（均为标量）可选的初始输出值 initial_output（默认为零）
        constexpr Lpf1st(Scalar dt, Scalar tc, const T &initial_output = T{})
        {
            alpha = dt / (tc + dt);
            prev_output = initial_output;
        }

        // 更新滤波器，传入当前输入值，返回滤波后的输出值
        constexpr T step(const T &input)
        {
            prev_output = input * alpha + prev_output * (Scalar(1) - alpha);
            return prev_output;
        }

        // 重置滤波器状态
        constexpr void reset(const T &value = T{})
        {
            prev_output = value;
        }

    private:
        Scalar alpha;
        T prev_output;
    };

    template <typename T>
    class Lpf2nd
    {
    public:
        using Scalar = scalar_t<T>;

        // 构造函数：传入采样周期 dt (s)、截止频率 fc (Hz)、阻尼比 zeta (默认 0.7071f Butterworth) 与初始输出
        Lpf2nd(Scalar dt, Scalar fc, Scalar zeta = Scalar(0.7071), const T &initial_output = T{})
        {
            // 1. 频偏预畸变计算 (Pre-warping)
            // K = tan(pi * fc * dt)
            const Scalar K = std::tan(std::numbers::pi_v<Scalar> * fc * dt);
            const Scalar K2 = K * K;
            const Scalar two_zeta_K = Scalar(2) * zeta * K;

            // 2. 双线性变换归一化分母
            const Scalar norm = Scalar(1) / (Scalar(1) + two_zeta_K + K2);

            // 3. 离散差分方程系数 (符合 y[n] + a1*y[n-1] + a2*y[n-2] = b0*x[n] + b1*x[n-1] + b2*x[n-2])
            b0 = K2 * norm;
            b1 = Scalar(2) * b0;
            b2 = b0;

            a1 = Scalar(2) * (K2 - Scalar(1)) * norm;
            a2 = (Scalar(1) - two_zeta_K + K2) * norm;

            // 4. 对齐初始稳态
            reset(initial_output);
        }

        // 更新滤波器：转置直接 II 型 (TDF-II)，数值稳定性极高且天然兼容 Vec 的四则运算
        constexpr T step(const T &input)
        {
            T output = input * b0 + s1;
            s1 = input * b1 - output * a1 + s2;
            s2 = input * b2 - output * a2;
            return output;
        }

        // 重置滤波器状态（使稳态阶跃响应无冲击，直接处于直流平衡）
        constexpr void reset(const T &value = T{})
        {
            // 直流稳态下: y = x, s1 = (1 - b0) * x, s2 = (b2 - a2) * x
            s1 = value * (Scalar(1) - b0);
            s2 = value * (b2 - a2);
        }

    private:
        // 标量系数
        Scalar b0{1}, b1{0}, b2{0};
        Scalar a1{0}, a2{0};

        // 状态变量（与信号类型 T 保持一致，支持标量与 Vec）
        T s1{};
        T s2{};
    };

    // 积分器
    template <typename T>
    class Integrator
    {
    public:
        using Scalar = scalar_t<T>;

        // 构造函数，传入采样周期 dt（标量）
        constexpr Integrator(Scalar dt, const T &initial_state = T{})
        {
            this->dt = dt;
            state = initial_state;
        }

        // 更新积分器，传入当前输入值，返回积分后的输出值
        constexpr T step(const T &input)
        {
            state += input * dt;
            return state;
        }

        // 重置积分器状态
        constexpr void reset(const T &value = T{})
        {
            state = value;
        }

    private:
        Scalar dt;
        T state;
    };

}