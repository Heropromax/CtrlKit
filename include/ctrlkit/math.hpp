#pragma once

#include <cmath>

#include <ctrlkit/ctrlkit.hpp>

namespace ctrlkit
{
    // ------------------ 三角函数

    template <typename T, unsigned int N>
    constexpr Vec<T, N> sin(const Vec<T, N> &x)
    {
        Vec<T, N> result;
        for (unsigned int i = 0; i < N; ++i)
            result[i] = std::sin(x[i]);
        return result;
    }

    template <typename T, unsigned int N>
    constexpr Vec<T, N> cos(const Vec<T, N> &x)
    {
        Vec<T, N> result;
        for (unsigned int i = 0; i < N; ++i)
            result[i] = std::cos(x[i]);
        return result;
    }

    template <typename T, unsigned int N>
    constexpr Vec<T, N> tan(const Vec<T, N> &x)
    {
        Vec<T, N> result;
        for (unsigned int i = 0; i < N; ++i)
            result[i] = std::tan(x[i]);
        return result;
    }

    template <typename T, unsigned int N>
    constexpr Vec<T, N> asin(const Vec<T, N> &x)
    {
        Vec<T, N> result;
        for (unsigned int i = 0; i < N; ++i)
            result[i] = std::asin(x[i]);
        return result;
    }

    template <typename T, unsigned int N>
    constexpr Vec<T, N> acos(const Vec<T, N> &x)
    {
        Vec<T, N> result;
        for (unsigned int i = 0; i < N; ++i)
            result[i] = std::acos(x[i]);
        return result;
    }

    template <typename T, unsigned int N>
    constexpr Vec<T, N> atan(const Vec<T, N> &x)
    {
        Vec<T, N> result;
        for (unsigned int i = 0; i < N; ++i)
            result[i] = std::atan(x[i]);
        return result;
    }

    // ------------------ 指数与对数

    template <typename T, unsigned int N>
    constexpr Vec<T, N> exp(const Vec<T, N> &x)
    {
        Vec<T, N> result;
        for (unsigned int i = 0; i < N; ++i)
            result[i] = std::exp(x[i]);
        return result;
    }

    template <typename T, unsigned int N>
    constexpr Vec<T, N> log(const Vec<T, N> &x)
    {
        Vec<T, N> result;
        for (unsigned int i = 0; i < N; ++i)
            result[i] = std::log(x[i]);
        return result;
    }

    // pow函数（向量，运行期标量指数）：对向量的每个元素进行幂运算。
    // 注意：指数是运行期值时只能调用运行库 std::pow，开销较大；
    // 若指数在编译期已知（整数），请改用下面的 pow<E> 重载。
    template <typename T, unsigned int N>
    constexpr Vec<T, N> pow(const Vec<T, N> &base, const T &exponent)
    {
        Vec<T, N> result;
        for (unsigned int i = 0; i < N; ++i)
            result[i] = std::pow(base[i], exponent);
        return result;
    }

    // pow函数（向量，编译期整数指数）：指数作为模板参数，展开成纯乘法，零运行库调用。
    // 调用形式：ctk::pow<2>(v)、ctk::pow<0>(v)、ctk::pow<-1>(v)。
    // 说明：展开是线性的，小指数最优；大指数可考虑平方求幂。
    template <int E, typename T, unsigned int N>
    constexpr Vec<T, N> pow(const Vec<T, N> &base)
    {
        // 特殊情况：指数为零，返回全 1 向量
        if constexpr (E == 0)
        {
            Vec<T, N> result;
            for (unsigned int i = 0; i < N; ++i)
                result[i] = static_cast<T>(1);
            return result;
        }
        else
        {
            constexpr int abs_exp = E < 0 ? -E : E;
            Vec<T, N> result = base;
            for (int e = 1; e < abs_exp; ++e)
                result *= base;

            // 如果指数为负数，则取倒数
            if constexpr (E < 0)
                return static_cast<T>(1) / result;
            // 如果指数为正数，则直接返回结果
            else
                return result;
        }
    }

} // namespace ctrlkit