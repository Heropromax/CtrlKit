// CtrlKit 冒烟测试：尽可能覆盖 Vec / Quat / Lpf1st 的所有对外接口。
// 编译期部分用 static_assert，运行期部分用 check() 累计通过/失败数。

#include <cmath>
#include <numbers>
#include <print>
#include <type_traits>

#include <ctrlkit/ctrlkit.hpp>
#include <ctrlkit/dynamic.hpp>
#include <ctrlkit/math.hpp>
#include <ctrlkit/quat.hpp>

namespace
{
    int g_pass = 0;
    int g_fail = 0;

    void check(bool ok, const char *name)
    {
        if (ok)
            ++g_pass;
        else
        {
            ++g_fail;
            std::println("  [FAIL] {}", name);
        }
    }

    template <typename T>
    bool approx(T a, T b, T eps = T(1e-5))
    {
        return std::abs(a - b) <= eps;
    }

    template <typename T, unsigned int N>
    bool approx_vec(const ctk::Vec<T, N> &a, const ctk::Vec<T, N> &b, T eps = T(1e-5))
    {
        return ctk::norm(a - b) <= eps;
    }

    // ============================ Vec ============================
    void test_vec()
    {
        using V3 = ctk::Vec<float, 3>;
        std::println("[Vec]");

        V3 a{1, 2, 3};
        V3 b{4, 5, 6};

        // 聚合初始化 + 下标
        check(a[0] == 1.0f && a[1] == 2.0f && a[2] == 3.0f, "aggregate init & operator[]");

        // 默认构造为全零
        V3 z;
        check(z[0] == 0.0f && z[1] == 0.0f && z[2] == 0.0f, "default init is zero");

        // 逐分量四则
        check((a + b) == (V3{5, 7, 9}), "vec + vec");
        check((b - a) == (V3{3, 3, 3}), "vec - vec");
        check((a * b) == (V3{4, 10, 18}), "vec * vec (Hadamard)");
        check((b / a) == (V3{4, 2.5f, 2}), "vec / vec");

        // 标量广播（右）
        check((a + 1.0f) == (V3{2, 3, 4}), "vec + scalar");
        check((a - 1.0f) == (V3{0, 1, 2}), "vec - scalar");
        check((a * 2.0f) == (V3{2, 4, 6}), "vec * scalar");
        check((a / 2.0f) == (V3{0.5f, 1, 1.5f}), "vec / scalar");

        // 标量广播（左，友元）
        check((1.0f + a) == (V3{2, 3, 4}), "scalar + vec");
        check((10.0f - a) == (V3{9, 8, 7}), "scalar - vec");
        check((2.0f * a) == (V3{2, 4, 6}), "scalar * vec");
        check((6.0f / a) == (V3{6, 3, 2}), "scalar / vec");

        // 一元
        check((-a) == (V3{-1, -2, -3}), "unary -");
        check((+a) == a, "unary +");

        // 复合赋值（vec op= vec）
        V3 c = a;
        c += b;
        check(c == (V3{5, 7, 9}), "vec += vec");
        c = a;
        c -= b;
        check(c == (V3{-3, -3, -3}), "vec -= vec");
        c = a;
        c *= b;
        check(c == (V3{4, 10, 18}), "vec *= vec");
        c = a;
        c /= b;
        check(approx(c[0], 0.25f) && approx(c[1], 0.4f) && approx(c[2], 0.5f), "vec /= vec");

        // 复合赋值（vec op= scalar）
        c = a;
        c += 1.0f;
        check(c == (V3{2, 3, 4}), "vec += scalar");
        c = a;
        c -= 1.0f;
        check(c == (V3{0, 1, 2}), "vec -= scalar");
        c = a;
        c *= 2.0f;
        check(c == (V3{2, 4, 6}), "vec *= scalar");
        c = a;
        c /= 2.0f;
        check(c == (V3{0.5f, 1, 1.5f}), "vec /= scalar");

        // 相等比较（!= 由 C++20 改写规则生成）
        check(a == (V3{1, 2, 3}), "operator==");
        check(a != b, "operator!=");

        // 自由函数
        check(approx(ctk::sum(a), 6.0f), "sum");
        check(approx(ctk::dot(a, b), 32.0f), "dot");
        check(approx(ctk::norm_sq(a), 14.0f), "norm_sq");
        check(approx(ctk::norm(V3{3, 4, 0}), 5.0f), "norm");
        check(ctk::cross(V3{1, 0, 0}, V3{0, 1, 0}) == (V3{0, 0, 1}), "cross");

        // 标量类型萃取
        static_assert(std::is_same_v<ctk::scalar_t<V3>, float>);
        static_assert(std::is_same_v<ctk::scalar_t<double>, double>);
        static_assert(std::is_same_v<ctk::scalar_t<ctk::Vec<double, 4>>, double>);

        // 变量模板常量：类型自动适配（无需 _f / _d 后缀）
        static_assert(std::is_same_v<std::remove_cv_t<decltype(ctk::deg2rad<float>)>, float>);
        static_assert(std::is_same_v<std::remove_cv_t<decltype(ctk::deg2rad<double>)>, double>);
        static_assert(ctk::pi<float> > 3.14f && ctk::pi<float> < 3.15f);
        static_assert(ctk::rad2deg<float> > 57.2f && ctk::rad2deg<float> < 57.4f);
        check(approx(ctk::deg2rad<double> * 180.0, ctk::pi<double>), "deg2rad * 180 == pi");

        // 编译期求值（constexpr）
        constexpr V3 ca{1, 2, 3};
        constexpr V3 cb{4, 5, 6};
        constexpr V3 cc = (ca + cb) * 2.0f - cb;
        static_assert(cc == V3{6, 9, 12});
        static_assert(ctk::dot(ca, cb) == 32.0f);
        static_assert(ctk::norm_sq(ca) == 14.0f);
        static_assert(ctk::sum(ca) == 6.0f);
        static_assert(ctk::cross(ca, cb) == V3{-3, 6, -3});
    }

    // ====================== Vec 元素类型转换 ======================
    void test_vec_cast()
    {
        using V3f = ctk::Vec<float, 3>;
        using V3d = ctk::Vec<double, 3>;
        using V3i = ctk::Vec<int, 3>;
        std::println("[Vec cast]");

        V3f vf{1.5f, -2.5f, 3.75f};

        // float -> double（提升）
        V3d vd = static_cast<V3d>(vf);
        check(approx(vd[0], 1.5) && approx(vd[1], -2.5) && approx(vd[2], 3.75), "float -> double");

        // float -> int（截断，与标量 static_cast 规则一致）
        V3i vi = static_cast<V3i>(vf);
        check(vi[0] == 1 && vi[1] == -2 && vi[2] == 3, "float -> int (truncate)");

        // int -> double
        V3d vd2 = static_cast<V3d>(V3i{1, 2, 3});
        check(vd2[0] == 1.0 && vd2[1] == 2.0 && vd2[2] == 3.0, "int -> double");

        // 与逐元素标量 static_cast 的结果一致
        check(vi[1] == static_cast<int>(vf[1]), "matches scalar static_cast");

        // 加转换运算符后仍是聚合类型，聚合初始化不受影响
        static_assert(std::is_aggregate_v<V3f>);
        check((V3f{1, 2, 3})[1] == 2.0f, "still aggregate-initializable");

        // 转换是显式的，不会隐式发生
        static_assert(!std::is_convertible_v<V3f, V3d>, "conversion is explicit");

        // 编译期可用
        constexpr V3f cvf{1.0f, 2.0f, 3.0f};
        constexpr V3d cvd = static_cast<V3d>(cvf);
        static_assert(cvd[2] == 3.0);
    }

    // ============================ Quat ============================
    void test_quat()
    {
        using Q = ctk::Quat<float>;
        using V3 = ctk::Vec<float, 3>;
        std::println("[Quat]");

        const float half_pi = std::numbers::pi_v<float> / 2.0f;

        // 默认构造为单位四元数
        Q id;
        check(id.w == 1.0f && id.x == 0.0f && id.y == 0.0f && id.z == 0.0f, "identity default");

        // 由欧拉角构造：绕 Z 转 90°
        Q qz = ctk::EulerZYX2Quat(V3{half_pi, 0.0f, 0.0f});
        check(approx(qz.w, std::cos(half_pi / 2.0f)) && approx(qz.z, std::sin(half_pi / 2.0f)),
              "EulerZYX2Quat (90deg about Z)");

        // 乘法：90° * 90° = 180°（w ≈ 0, z ≈ 1）
        Q q180 = qz * qz;
        check(approx(std::abs(q180.w), 0.0f) && approx(std::abs(q180.z), 1.0f), "q * q = 180deg");

        // 旋转矢量：(1,0,0) 绕 Z 转 90° → (0,1,0)
        check(approx_vec(ctk::rotvec(qz, V3{1, 0, 0}), V3{0, 1, 0}), "rotvec (90deg about Z)");

        // 共轭应抵消旋转
        Q qc = qz.conj();
        V3 v0{1, 2, 3};
        check(approx_vec(ctk::rotvec(qc, ctk::rotvec(qz, v0)), v0), "conj undoes rotation");

        // 标量运算
        check(approx((qz * 2.0f).w, qz.w * 2.0f), "quat * scalar");
        check(approx((2.0f * qz).z, qz.z * 2.0f), "scalar * quat");
        check(approx((qz / 2.0f).w, qz.w * 0.5f), "quat / scalar");
        check(approx((1.0f / qz).w, 1.0f / qz.w), "scalar / quat");
        Q qm = qz;
        qm *= 2.0f;
        check(approx(qm.w, qz.w * 2.0f), "quat *= scalar");
        qm = qz;
        qm /= 2.0f;
        check(approx(qm.w, qz.w * 0.5f), "quat /= scalar");

        // 模长与归一化
        check(approx(ctk::norm_sq(qz), 1.0f), "unit norm_sq");
        check(approx(ctk::norm(qz), 1.0f), "unit norm");
        Q qn = ctk::normalize(Q{0.0f, 0.0f, 0.0f, 2.0f});
        check(approx(ctk::norm(qn), 1.0f), "normalize");
        check(ctk::normalize(Q{0.0f, 0.0f, 0.0f, 0.0f}).w == 1.0f, "normalize(zero) -> identity");

        // 欧拉角往返（弧度 / 度数）
        V3 euler_in{0.3f, -0.2f, 0.1f};
        check(approx_vec(ctk::Quat2EulerZYX(ctk::EulerZYX2Quat(euler_in)), euler_in, 1e-4f),
              "Euler round trip (rad)");

        V3 deg_in{30.0f, -15.0f, 10.0f};
        check(approx_vec(ctk::Quat2EulerZYXDeg(ctk::EulerZYXDeg2Quat(deg_in)), deg_in, 1e-2f),
              "Euler round trip (deg)");

        // 编译期：四元数乘法（i * j = k）
        constexpr ctk::Quat<float> qi{0, 1, 0, 0};
        constexpr ctk::Quat<float> qj{0, 0, 1, 0};
        constexpr ctk::Quat<float> qk = qi * qj;
        static_assert(qk.w == 0.0f && qk.x == 0.0f && qk.y == 0.0f && qk.z == 1.0f);
    }

    // ========================= Lpf1st =========================
    void test_lpf()
    {
        using V3 = ctk::Vec<float, 3>;
        std::println("[Lpf1st]");

        const double dt = 0.01, tc = 0.1;
        const double alpha = dt / (tc + dt);

        // 标量：第一步输出 = alpha * input
        ctk::Lpf1st<double> f(dt, tc);
        check(std::abs(f.step(1.0) - alpha) < 1e-12, "scalar: first step = alpha * input");

        // 标量：阶跃输入最终收敛到 1
        ctk::Lpf1st<double> f2(dt, tc);
        double y = 0.0;
        for (int i = 0; i < 1000; ++i)
            y = f2.step(1.0);
        check(std::abs(y - 1.0) < 1e-3, "scalar: converges to 1");

        // 标量：输入 0 输出 0
        ctk::Lpf1st<double> f3(dt, tc);
        check(f3.step(0.0) == 0.0, "scalar: zero in -> zero out");

        // reset
        ctk::Lpf1st<double> f4(dt, tc);
        f4.step(5.0);
        f4.reset(0.0);
        check(f4.step(0.0) == 0.0, "reset");

        // 向量：三轴共用同一 alpha，第一步 = alpha * input
        ctk::Lpf1st<V3> vf(static_cast<float>(dt), static_cast<float>(tc));
        const float alphaf = static_cast<float>(alpha);
        V3 first = vf.step(V3{1.0f, 2.0f, 3.0f});
        check(approx_vec(first, V3{alphaf, 2.0f * alphaf, 3.0f * alphaf}), "vec: first step = alpha * input");

        // 向量：逐步收敛到输入
        ctk::Lpf1st<V3> vf2(static_cast<float>(dt), static_cast<float>(tc));
        const V3 target{1.0f, -2.0f, 3.0f};
        V3 out{};
        for (int i = 0; i < 2000; ++i)
            out = vf2.step(target);
        check(approx_vec(out, target, 1e-3f), "vec: converges to input on all axes");

        // 初始输出：从指定值起步（可减少启动瞬态）
        ctk::Lpf1st<double> f5(dt, tc, 10.0);
        check(approx(f5.step(10.0), 10.0), "initial_output: steady at initial value");

        // 初始值非零 + 零输入 → 按 (1 - alpha) 衰减
        ctk::Lpf1st<double> f6(0.5, 0.5, 1.0); // alpha = 0.5
        check(approx(f6.step(0.0), 0.5), "initial_output decays toward 0");

        // 向量初始输出
        ctk::Lpf1st<V3> vf3(0.5f, 0.5f, V3{1.0f, 2.0f, 3.0f});
        check(approx_vec(vf3.step(V3{1.0f, 2.0f, 3.0f}), V3{1.0f, 2.0f, 3.0f}), "vec initial_output");

        // 滤波器可用 constexpr 构造与求值（在 constexpr 函数内使用非 const 局部对象）
        constexpr double first_step = []
        {
            ctk::Lpf1st<double> f(0.01, 0.1);
            return f.step(1.0);
        }();
        static_assert(first_step > 0.0 && first_step < 1.0);

        // 初始输出同样可用于编译期
        constexpr double init_step = []
        {
            ctk::Lpf1st<double> f(0.5, 0.5, 1.0);
            return f.step(0.0);
        }();
        static_assert(init_step == 0.5);
    }

    // ========================= Integrator =========================
    void test_integrator()
    {
        using V3 = ctk::Vec<float, 3>;
        std::println("[Integrator]");

        // 标量：常数输入 a，n 步后 = n * a * dt
        ctk::Integrator<double> i1(0.5);
        double y = 0.0;
        for (int k = 0; k < 10; ++k)
            y = i1.step(2.0);
        check(approx(y, 10.0 * 2.0 * 0.5), "scalar: const input -> n*a*dt");

        // 标量：第一步 = input * dt
        ctk::Integrator<double> i2(0.25);
        check(approx(i2.step(4.0), 1.0), "scalar: first step = input*dt");

        // 标量：零输入一直为 0
        ctk::Integrator<double> i3(0.1);
        for (int k = 0; k < 5; ++k)
            i3.step(0.0);
        check(i3.step(0.0) == 0.0, "scalar: zero input stays zero");

        // 标量：正负输入对冲
        ctk::Integrator<double> i4(0.1);
        i4.step(1.0);
        check(approx(i4.step(-1.0), 0.0), "scalar: positive/negative cancel");

        // reset 可设初值
        ctk::Integrator<double> i5(0.1);
        i5.reset(5.0);
        check(approx(i5.step(0.0), 5.0), "reset sets initial state");

        // 向量：逐轴独立积分，共用同一个 dt
        ctk::Integrator<V3> vi(0.5f);
        const V3 iv{1.0f, 2.0f, -3.0f};
        V3 out{};
        for (int k = 0; k < 4; ++k)
            out = vi.step(iv);
        check(approx_vec(out, V3{2.0f, 4.0f, -6.0f}), "vec: per-axis integration");

        // 编译期可用
        constexpr double acc = []
        {
            ctk::Integrator<double> it(0.5);
            it.step(2.0);
            it.step(2.0);
            return it.step(2.0);
        }();
        static_assert(acc == 3.0);
    }

    // ============================ Lpf2nd ============================
    void test_lpf2()
    {
        using V3 = ctk::Vec<float, 3>;
        std::println("[Lpf2nd]");

        const double dt = 0.001, fc = 10.0;
        const double zeta = 1.0 / std::sqrt(2.0); // Butterworth

        // 直流增益 = 1：常数输入最终收敛到该常数
        ctk::Lpf2nd<double> f(dt, fc, zeta);
        double y = 0.0;
        for (int n = 0; n < 20000; ++n)
            y = f.step(1.0);
        check(std::abs(y - 1.0) < 1e-9, "DC gain = 1");

        // 初始稳态：以 5.0 构造后，输入恒为 5.0 时应保持 5.0（无启动冲击）
        ctk::Lpf2nd<double> f2(dt, fc, zeta, 5.0);
        bool steady = true;
        for (int n = 0; n < 200; ++n)
            steady = steady && std::abs(f2.step(5.0) - 5.0) < 1e-10;
        check(steady, "initial steady state: no startup transient");

        // 二阶 Butterworth 阶跃响应超调 ≈ exp(-pi) ≈ 4.32%
        ctk::Lpf2nd<double> f3(dt, fc, zeta);
        double peak = 0.0;
        for (int n = 0; n < 5000; ++n)
        {
            const double v = f3.step(1.0);
            if (v > peak)
                peak = v;
        }
        check(peak > 1.02 && peak < 1.07, "step overshoot ~4.3%");

        // 频率响应：fc 处应为 -3dB（预畸变保证奇点正好落在 fc）
        {
            const double w = 2.0 * ctk::pi<double> * fc;
            ctk::Lpf2nd<double> flt(dt, fc, zeta);
            double amp = 0.0;
            for (int n = 0; n < 5000; ++n)
            {
                const double yn = flt.step(std::sin(w * n * dt));
                if (n >= 4000 && std::abs(yn) > amp)
                    amp = std::abs(yn);
            }
            check(std::abs(amp - 1.0 / std::sqrt(2.0)) < 0.02, "gain at fc is -3dB");
        }

        // 远低于 fc：增益 ≈ 1
        {
            const double w = 2.0 * ctk::pi<double> * (fc / 20.0);
            ctk::Lpf2nd<double> flt(dt, fc, zeta);
            double amp = 0.0;
            for (int n = 0; n < 40000; ++n)
            {
                const double yn = flt.step(std::sin(w * n * dt));
                if (n >= 36000 && std::abs(yn) > amp)
                    amp = std::abs(yn);
            }
            check(std::abs(amp - 1.0) < 0.01, "gain well below fc is ~1");
        }

        // 向量：三轴共用同一组系数
        ctk::Lpf2nd<V3> vf(0.001f, 10.0f, 0.70711f);
        V3 out{};
        for (int n = 0; n < 20000; ++n)
            out = vf.step(V3{1.0f, 2.0f, 3.0f});
        check(approx_vec(out, V3{1.0f, 2.0f, 3.0f}, 1e-4f), "vec: DC gain = 1 on all axes");
    }

    // =========================== math.hpp ===========================
    void test_math()
    {
        using V3 = ctk::Vec<float, 3>;
        std::println("[Math]");

        const float pif = ctk::pi<float>;
        const V3 angles{0.0f, pif / 2.0f, pif};

        check(approx_vec(ctk::sin(angles), V3{0.0f, 1.0f, 0.0f}, 1e-6f), "sin");
        check(approx_vec(ctk::cos(angles), V3{1.0f, 0.0f, -1.0f}, 1e-6f), "cos");
        check(approx_vec(ctk::tan(V3{0.0f, 0.0f, 0.0f}), V3{0.0f, 0.0f, 0.0f}), "tan(0) = 0");
        check(approx_vec(ctk::asin(V3{0.0f, 1.0f, -1.0f}), V3{0.0f, pif / 2.0f, -pif / 2.0f}, 1e-6f), "asin");
        check(approx_vec(ctk::acos(V3{1.0f, 0.0f, -1.0f}), V3{0.0f, pif / 2.0f, pif}, 1e-6f), "acos");
        check(approx_vec(ctk::atan(V3{0.0f, 1.0f, -1.0f}), V3{0.0f, pif / 4.0f, -pif / 4.0f}, 1e-6f), "atan");

        const V3 exps{1.0f, std::exp(1.0f), std::exp(2.0f)};
        check(approx_vec(ctk::exp(V3{0.0f, 1.0f, 2.0f}), exps, 1e-6f), "exp");
        check(approx_vec(ctk::log(exps), V3{0.0f, 1.0f, 2.0f}, 1e-6f), "log");

        // 逐元素结果与标量 std 一致
        const V3 xs{-0.5f, 0.25f, 1.5f};
        check(approx_vec(ctk::sin(xs), V3{std::sin(-0.5f), std::sin(0.25f), std::sin(1.5f)}, 1e-7f),
              "sin matches std per element");

        // 注：MSVC 的 std::sin / std::log 等目前不是 constexpr 函数，
        // 因此 math.hpp 中虽然标了 constexpr，也只能运行期求值（见审核说明）。
        check(ctk::sin(V3{0.0f, 0.0f, 0.0f})[1] == 0.0f, "sin(0) == 0");
    }
}

int main()
{
    std::println("==== CtrlKit tests ====");

    test_vec();
    test_vec_cast();
    test_quat();
    test_lpf();
    test_lpf2();
    test_integrator();
    test_math();

    std::println("=======================");
    std::println("passed: {}, failed: {}", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}