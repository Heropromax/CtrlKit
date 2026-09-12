#pragma once

#include <cmath>
#include <numbers>
#include <type_traits>

#include <ctrlkit/ctrlkit.hpp>

namespace ctrlkit
{
    template <typename T>
    struct Quat
    {
        static_assert(std::is_floating_point_v<T>, "CtrlKit: Quat must be a floating point type");
        T w = 1;
        T x = 0;
        T y = 0;
        T z = 0;

        // ------------------ 四元数运算
        // 四元数乘法
        constexpr Quat operator*(const Quat &rhs) const
        {
            return Quat{
                w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
                w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
                w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
                w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w};
        }

        // 四元数与标量乘法
        constexpr Quat operator*(const T &rhs) const
        {
            return Quat{w * rhs, x * rhs, y * rhs, z * rhs};
        }

        // 四元数与标量除法
        constexpr Quat operator/(const T &rhs) const
        {
            return Quat{w / rhs, x / rhs, y / rhs, z / rhs};
        }

        // ------------------ 标量在左的运算（友元，支持 s * q / s / q 这类写法）

        friend constexpr Quat operator*(const T &lhs, const Quat &rhs)
        {
            return Quat{lhs * rhs.w, lhs * rhs.x, lhs * rhs.y, lhs * rhs.z};
        }

        friend constexpr Quat operator/(const T &lhs, const Quat &rhs)
        {
            return Quat{lhs / rhs.w, lhs / rhs.x, lhs / rhs.y, lhs / rhs.z};
        }

        // ------------------ 复合赋值运算（四元数 op= 标量）

        constexpr Quat &operator*=(const T &rhs)
        {
            w *= rhs;
            x *= rhs;
            y *= rhs;
            z *= rhs;
            return *this;
        }

        constexpr Quat &operator/=(const T &rhs)
        {
            w /= rhs;
            x /= rhs;
            y /= rhs;
            z /= rhs;
            return *this;
        }

        // 四元数共轭
        constexpr Quat conj() const
        {
            return Quat{w, -x, -y, -z};
        }
    };

    template <typename T>
    constexpr T norm_sq(const Quat<T> &q)
    {
        return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    }

    template <typename T>
    constexpr T norm(const Quat<T> &q)
    {
        return std::sqrt(norm_sq(q));
    }

    // 四元数归一化（若模长为零，则返回单位四元数）
    template <typename T>
    constexpr Quat<T> normalize(const Quat<T> &q)
    {
        T n = norm(q);
        if (n == T(0))
            return Quat<T>{1, 0, 0, 0}; // 返回单位四元数
        return Quat<T>{q.w / n, q.x / n, q.y / n, q.z / n};
    }

    // 四元数旋转矢量（仅支持三维向量和单位四元数）
    // 采用等价的高效公式：v' = v + 2w*(u × v) + 2*(u × (u × v))，u = (x, y, z) 为虚部。
    // 相比 q * v_quat * q_conj（两次四元数乘法），此处只需两次叉乘，乘法次数约减半。
    template <typename T>
    constexpr Vec<T, 3> rotvec(const Quat<T> &q, const Vec<T, 3> &v)
    {
        // t = 2 * (u × v)
        const T tx = T(2) * (q.y * v[2] - q.z * v[1]);
        const T ty = T(2) * (q.z * v[0] - q.x * v[2]);
        const T tz = T(2) * (q.x * v[1] - q.y * v[0]);

        // v' = v + w * t + (u × t)
        return Vec<T, 3>{
            v[0] + q.w * tx + (q.y * tz - q.z * ty),
            v[1] + q.w * ty + (q.z * tx - q.x * tz),
            v[2] + q.w * tz + (q.x * ty - q.y * tx)};
    }

    // 从四元数转换为欧拉角（ZYX顺序）
    template <typename T>
    constexpr Vec<T, 3> Quat2EulerZYX(const Quat<T> &q)
    {
        T sinr_cosp = T(2) * (q.w * q.x + q.y * q.z);
        T cosr_cosp = T(1) - T(2) * (q.x * q.x + q.y * q.y);
        T roll = std::atan2(sinr_cosp, cosr_cosp);

        T sinp = T(2) * (q.w * q.y - q.z * q.x);
        T pitch;
        if (std::abs(sinp) >= T(1))
            pitch = std::copysign(std::numbers::pi_v<T> / T(2), sinp); // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        T siny_cosp = T(2) * (q.w * q.z + q.x * q.y);
        T cosy_cosp = T(1) - T(2) * (q.y * q.y + q.z * q.z);
        T yaw = std::atan2(siny_cosp, cosy_cosp);

        return Vec<T, 3>{yaw, pitch, roll};
    }

    // 从欧拉角（ZYX顺序）转换为四元数
    template <typename T>
    constexpr Quat<T> EulerZYX2Quat(const Vec<T, 3> &eulerZYX)
    {
        T cy = std::cos(eulerZYX[0] * T(0.5));
        T sy = std::sin(eulerZYX[0] * T(0.5));
        T cp = std::cos(eulerZYX[1] * T(0.5));
        T sp = std::sin(eulerZYX[1] * T(0.5));
        T cr = std::cos(eulerZYX[2] * T(0.5));
        T sr = std::sin(eulerZYX[2] * T(0.5));

        return Quat<T>{
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy};
    }

}

namespace ctrlkit
{
    // ------------------ 语法糖，只对外使用， 对内依旧使用稳定的std 库
    using Quatf = Quat<float>;
    using Quatd = Quat<double>;

}