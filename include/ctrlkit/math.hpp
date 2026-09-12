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

}