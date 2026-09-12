#pragma once

#include <cmath>
#include <concepts>
#include <numbers>
#include <type_traits>

namespace ctrlkit
{
    template <typename T, unsigned int N>
    struct Vec
    {
        static_assert(N >= 2, "CtrlKit: Vec dimension must be at least 2");
        T data[N] = {};

        // 2. 像普通数组一样读写数据：vec[0] = 1.0f;
        constexpr T &operator[](unsigned int i) { return data[i]; }
        constexpr const T &operator[](unsigned int i) const { return data[i]; }

        // ------------------ 向量四则运算

        constexpr Vec operator+(const Vec &rhs) const
        {
            Vec res;
            for (unsigned int i = 0; i < N; ++i)
                res.data[i] = data[i] + rhs.data[i];
            return res;
        }
        constexpr Vec operator-(const Vec &rhs) const
        {
            Vec res;
            for (unsigned int i = 0; i < N; ++i)
                res.data[i] = data[i] - rhs.data[i];
            return res;
        }

        constexpr Vec operator*(const Vec &rhs) const
        {
            Vec res;
            for (unsigned int i = 0; i < N; ++i)
                res.data[i] = data[i] * rhs.data[i];
            return res;
        }
        constexpr Vec operator/(const Vec &rhs) const
        {
            Vec res;
            for (unsigned int i = 0; i < N; ++i)
                res.data[i] = data[i] / rhs.data[i];
            return res;
        }

        // ------------------ 标量广播向量四则运算

        constexpr Vec operator+(const T &rhs) const
        {
            Vec res;
            for (unsigned int i = 0; i < N; ++i)
                res.data[i] = data[i] + rhs;
            return res;
        }

        constexpr Vec operator-(const T &rhs) const
        {
            Vec res;
            for (unsigned int i = 0; i < N; ++i)
                res.data[i] = data[i] - rhs;
            return res;
        }

        constexpr Vec operator*(const T &rhs) const
        {
            Vec res;
            for (unsigned int i = 0; i < N; ++i)
                res.data[i] = data[i] * rhs;
            return res;
        }

        constexpr Vec operator/(const T &rhs) const
        {
            Vec res;
            for (unsigned int i = 0; i < N; ++i)
                res.data[i] = data[i] / rhs;
            return res;
        }

        // ------------------ 标量在左边的运算（友元，支持 lhs + vec 这类写法）

        friend constexpr Vec operator+(const T &lhs, const Vec &rhs)
        {
            Vec res;
            for (unsigned int i = 0; i < N; ++i)
                res.data[i] = lhs + rhs.data[i];
            return res;
        }

        friend constexpr Vec operator-(const T &lhs, const Vec &rhs)
        {
            Vec res;
            for (unsigned int i = 0; i < N; ++i)
                res.data[i] = lhs - rhs.data[i];
            return res;
        }

        friend constexpr Vec operator*(const T &lhs, const Vec &rhs)
        {
            Vec res;
            for (unsigned int i = 0; i < N; ++i)
                res.data[i] = lhs * rhs.data[i];
            return res;
        }

        friend constexpr Vec operator/(const T &lhs, const Vec &rhs)
        {
            Vec res;
            for (unsigned int i = 0; i < N; ++i)
                res.data[i] = lhs / rhs.data[i];
            return res;
        }

        // ------------------ 一元运算

        constexpr Vec operator+() const
        {
            return *this;
        }

        constexpr Vec operator-() const
        {
            Vec res;
            for (unsigned int i = 0; i < N; ++i)
                res.data[i] = -data[i];
            return res;
        }

        // ------------------ 复合赋值运算（vec op= vec）

        constexpr Vec &operator+=(const Vec &rhs)
        {
            for (unsigned int i = 0; i < N; ++i)
                data[i] += rhs.data[i];
            return *this;
        }

        constexpr Vec &operator-=(const Vec &rhs)
        {
            for (unsigned int i = 0; i < N; ++i)
                data[i] -= rhs.data[i];
            return *this;
        }

        constexpr Vec &operator*=(const Vec &rhs)
        {
            for (unsigned int i = 0; i < N; ++i)
                data[i] *= rhs.data[i];
            return *this;
        }

        constexpr Vec &operator/=(const Vec &rhs)
        {
            for (unsigned int i = 0; i < N; ++i)
                data[i] /= rhs.data[i];
            return *this;
        }

        // ------------------ 复合赋值运算（vec op= scalar）

        constexpr Vec &operator+=(const T &rhs)
        {
            for (unsigned int i = 0; i < N; ++i)
                data[i] += rhs;
            return *this;
        }

        constexpr Vec &operator-=(const T &rhs)
        {
            for (unsigned int i = 0; i < N; ++i)
                data[i] -= rhs;
            return *this;
        }

        constexpr Vec &operator*=(const T &rhs)
        {
            for (unsigned int i = 0; i < N; ++i)
                data[i] *= rhs;
            return *this;
        }

        constexpr Vec &operator/=(const T &rhs)
        {
            for (unsigned int i = 0; i < N; ++i)
                data[i] /= rhs;
            return *this;
        }

        // ------------------ 相等比较（!= 由 C++20 改写规则自动生成）

        friend constexpr bool operator==(const Vec &lhs, const Vec &rhs)
        {
            for (unsigned int i = 0; i < N; ++i)
                if (!(lhs.data[i] == rhs.data[i]))
                    return false;
            return true;
        }

        // ------------------ 元素类型转换（显式，等价于逐元素 static_cast）

        // 用法：static_cast<Vec<U, N>>(v)，逐元素按标量 static_cast 的规则转换
        // （float->int 会截断，与标量语义一致）。
        // 标记 explicit：既能用 static_cast 显式触发，又不会引入隐式转换带来的重载歧义，
        // 且转换运算符不是构造函数，故 Vec 仍保持聚合类型。
        template <typename U>
        constexpr explicit operator Vec<U, N>() const
        {
            Vec<U, N> res{};
            for (unsigned int i = 0; i < N; ++i)
                res[i] = static_cast<U>(data[i]);
            return res;
        }
    };

    // ------------------ 标量类型萃取（scalar_of / scalar_t）

    // 主模板：标量类型（float、double 等）的标量即其自身
    template <typename T>
    struct scalar_of
    {
        using type = T;
    };

    // 偏特化：向量的标量是其元素类型
    template <typename U, unsigned int N>
    struct scalar_of<Vec<U, N>>
    {
        using type = U;
    };

    // 对外别名：scalar_t<T>，调用处无需再写 typename / ::type
    template <typename T>
    using scalar_t = typename scalar_of<T>::type;

    // ------------------ 向量运算的自由函数

    // 向量和
    template <typename T, unsigned int N>
    constexpr T sum(const Vec<T, N> &vec)
    {
        T s = 0;
        for (unsigned int i = 0; i < N; ++i)
            s += vec[i];
        return s;
    }

    // 向量点积
    template <typename T, unsigned int N>
    constexpr T dot(const Vec<T, N> &lhs, const Vec<T, N> &rhs)
    {
        return sum(lhs * rhs);
    }

    // 向量模长平方
    template <typename T, unsigned int N>
    constexpr T norm_sq(const Vec<T, N> &vec)
    {
        return dot(vec, vec);
    }

    // 向量模长
    template <typename T, unsigned int N>
    constexpr T norm(const Vec<T, N> &vec)
    {
        return std::sqrt(dot(vec, vec));
    }

    // 向量叉乘（仅支持三维向量）
    template <typename T>
    constexpr Vec<T, 3> cross(const Vec<T, 3> &lhs, const Vec<T, 3> &rhs)
    {
        return Vec<T, 3>{
            lhs[1] * rhs[2] - lhs[2] * rhs[1],
            lhs[2] * rhs[0] - lhs[0] * rhs[2],
            lhs[0] * rhs[1] - lhs[1] * rhs[0]};
    }

}

namespace ctrlkit
{
    // ------------------ 语法糖，只对外使用， 对内依旧使用稳定的std 库

    using Vec3f = Vec<float, 3>;
    using Vec3d = Vec<double, 3>;

    // 目前支持两种pi定义，看那种用的爽后续确定API

    // 仅支持浮点类型的 pi, 不支持Vec，避免误用
    template <typename T>
        requires std::floating_point<T>
    constexpr T pi = std::numbers::pi_v<T>;

    // 专门用于浮点数的 pi 常量
    constexpr float pi_f = std::numbers::pi_v<float>;
    constexpr double pi_d = std::numbers::pi_v<double>;

    template <typename T>
    constexpr T deg2rad(T deg)
    {
        using Scalar = scalar_t<T>;
        static_assert(std::is_floating_point_v<Scalar>, "deg2rad: T must be a floating-point type");
        return deg * std::numbers::pi_v<Scalar> / Scalar(180);
    }

    template <typename T>
    constexpr T rad2deg(T rad)
    {
        using Scalar = scalar_t<T>;
        static_assert(std::is_floating_point_v<Scalar>, "rad2deg: T must be a floating-point type");
        return rad * Scalar(180) / std::numbers::pi_v<Scalar>;
    }

}

namespace ctk = ctrlkit;