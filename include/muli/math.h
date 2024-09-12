// Simple linear math library
// Highly inspired by Box2d, glm math codes

#pragma once

#include "format.h"
#include "types.h"

namespace muli
{

constexpr float pi = 3.14159265359f;
constexpr float epsilon = FLT_EPSILON;
constexpr float max_value = FLT_MAX;

struct Vec2;
struct Vec3;
struct Vec4;
struct Mat2;
struct Mat3;
struct Mat4;
struct Rotation;
struct Transform;
struct Sweep;

enum Identity
{
    identity
};

struct Vec2
{
    float x, y;

    Vec2() = default;

    constexpr explicit Vec2(float s)
        : x{ s }
        , y{ s }
    {
    }

    constexpr Vec2(float x, float y)
        : x{ x }
        , y{ y }
    {
    }

    void SetZero()
    {
        x = 0.0f;
        y = 0.0f;
    }

    void Set(float s)
    {
        x = s;
        y = s;
    }

    void Set(float _x, float _y)
    {
        x = _x;
        y = _y;
    }

    float operator[](int32 i) const
    {
        return (&x)[i];
    }

    float& operator[](int32 i)
    {
        return (&x)[i];
    }

    Vec2 operator-() const
    {
        return Vec2{ -x, -y };
    }

    void operator+=(const Vec2& v)
    {
        x += v.x;
        y += v.y;
    }

    void operator-=(const Vec2& v)
    {
        x -= v.x;
        y -= v.y;
    }

    void operator+=(float s)
    {
        x += s;
        y += s;
    }

    void operator-=(float s)
    {
        x -= s;
        y -= s;
    }

    void operator*=(float s)
    {
        x *= s;
        y *= s;
    }

    void operator/=(float s)
    {
        operator*=(1.0f / s);
    }

    float Length() const
    {
        return sqrtf(x * x + y * y);
    }

    float Length2() const
    {
        return x * x + y * y;
    }

    float Normalize()
    {
        float length = Length();
        assert(length > 0.0f);

        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;

        return length;
    }

    float NormalizeSafe()
    {
        float length = Length();
        if (length < epsilon)
        {
            return 0.0f;
        }

        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;

        return length;
    }

    // Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
    // return cross(1, *this);
    Vec2 Skew() const
    {
        return Vec2{ -y, x };
    }

    std::string ToString() const
    {
        return FormatString("%.4f\t%.4f", x, y);
    }

    static const Vec2 zero;
};

constexpr inline Vec2 Vec2::zero{ 0.0f };

struct Vec3
{
    float x, y, z;

    Vec3() = default;

    constexpr explicit Vec3(float s)
        : x{ s }
        , y{ s }
        , z{ s }
    {
    }

    constexpr Vec3(float x, float y, float z)
        : x{ x }
        , y{ y }
        , z{ z }
    {
    }

    Vec3(const Vec2& v)
        : x{ v.x }
        , y{ v.y }
        , z{ 0.0f }
    {
    }

    void SetZero()
    {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }

    void Set(float s)
    {
        x = s;
        y = s;
        z = s;
    }

    void Set(float _x, float _y, float _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }

    float operator[](int32 i) const
    {
        return (&x)[i];
    }

    float& operator[](int32 i)
    {
        return (&x)[i];
    }

    Vec3 operator-() const
    {
        return Vec3{ -x, -y, -z };
    }

    void operator+=(const Vec3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    void operator-=(const Vec3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    void operator+=(float s)
    {
        x += s;
        y += s;
        z += s;
    }

    void operator-=(float s)
    {
        x -= s;
        y -= s;
        z -= s;
    }

    void operator*=(float s)
    {
        x *= s;
        y *= s;
        z *= s;
    }

    void operator/=(float s)
    {
        operator*=(1.0f / s);
    }

    float Length() const
    {
        return sqrtf(x * x + y * y + z * z);
    }

    float Length2() const
    {
        return x * x + y * y + z * z;
    }

    float Normalize()
    {
        float length = Length();
        assert(length > 0.0f);

        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;
        z *= invLength;

        return length;
    }

    float NormalizeSafe()
    {
        float length = Length();
        if (length < epsilon)
        {
            return 0.0f;
        }

        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;
        z *= invLength;

        return length;
    }

    std::string ToString() const
    {
        return FormatString("%.4f\t%.4f\t%.4f", x, y, z);
    }

    static const Vec3 zero;
};

constexpr inline Vec3 Vec3::zero{ 0.0f };

struct Vec4
{
    float x, y, z, w;

    Vec4() = default;

    constexpr Vec4(float v, float w)
        : x{ v }
        , y{ v }
        , z{ v }
        , w{ w }
    {
    }

    constexpr Vec4(float x, float y, float z, float w)
        : x{ x }
        , y{ y }
        , z{ z }
        , w{ w }
    {
    }

    constexpr Vec4(const Vec3& v, float w)
        : x{ v.x }
        , y{ v.y }
        , z{ v.z }
        , w{ w }
    {
    }

    void SetZero()
    {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        w = 0.0f;
    }

    void Set(float s)
    {
        x = s;
        y = s;
        z = s;
        w = s;
    }

    void Set(float _x, float _y, float _z, float _w)
    {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
    }

    float operator[](int32 i) const
    {
        return (&x)[i];
    }

    float& operator[](int32 i)
    {
        return (&x)[i];
    }

    Vec4 operator-() const
    {
        return Vec4{ -x, -y, -z, -w };
    }

    void operator+=(const Vec4& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        w += v.w;
    }

    void operator-=(const Vec4& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        w -= v.w;
    }

    void operator+=(float s)
    {
        x += s;
        y += s;
        z += s;
        w += s;
    }

    void operator-=(float s)
    {
        x -= s;
        y -= s;
        z -= s;
        w -= s;
    }

    void operator*=(float s)
    {
        x *= s;
        y *= s;
        z *= s;
        w *= s;
    }

    void operator/=(float s)
    {
        operator*=(1.0f / s);
    }

    std::string ToString() const
    {
        return FormatString("%.4f\t%.4f\t%.4f\t%.4f", x, y, z, w);
    }

    static const Vec4 zero;
};

constexpr inline Vec4 Vec4::zero{ 0.0f, 0.0f };

struct Quat
{
    Quat() = default;

    Quat(Identity)
        : Quat(1.0f)
    {
    }

    Quat(float x, float y, float z, float w)
        : x{ x }
        , y{ y }
        , z{ z }
        , w{ w }
    {
    }

    constexpr explicit Quat(float w)
        : x{ 0.0f }
        , y{ 0.0f }
        , z{ 0.0f }
        , w{ w }
    {
    }

    Quat(const Mat3& m);

    Quat(const Vec3& dir, const Vec3& up);

    // Axis must be normalized
    Quat(float angle, const Vec3& unitAxis)
    {
        float halfAngle = angle * 0.5f;

        float s = sinf(halfAngle);
        x = unitAxis.x * s;
        y = unitAxis.y * s;
        z = unitAxis.z * s;
        w = cosf(halfAngle);
    }

    Quat operator-()
    {
        return Quat{ -x, -y, -z, -w };
    }

    Quat operator*(float s) const
    {
        return Quat{ x * s, y * s, z * s, w * s };
    }

    bool IsIdentity() const
    {
        return x == 0.0f && y == 0.0f && z == 0.0f && w == 1.0f;
    }

    // Magnitude
    float Length() const
    {
        return sqrtf(x * x + y * y + z * z + w * w);
    }

    float Length2() const
    {
        return x * x + y * y + z * z + w * w;
    }

    float Normalize()
    {
        float length = Length();
        assert(length > 0.0f);

        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;
        z *= invLength;
        w *= invLength;

        return length;
    }

    Quat GetConjugate() const
    {
        return Quat{ -x, -y, -z, w };
    }

    Vec3 GetImaginaryPart() const
    {
        return Vec3{ x, y, z };
    }

    // Optimized qvq'
    Vec3 Rotate(const Vec3& v) const
    {
        float vx = 2.0f * v.x;
        float vy = 2.0f * v.y;
        float vz = 2.0f * v.z;
        float w2 = w * w - 0.5f;

        float dot2 = (x * vx + y * vy + z * vz);

        return Vec3(
            (vx * w2 + (y * vz - z * vy) * w + x * dot2), (vy * w2 + (z * vx - x * vz) * w + y * dot2),
            (vz * w2 + (x * vy - y * vx) * w + z * dot2)
        );
    }

    Vec3 RotateInv(const Vec3& v) const
    {
        float vx = 2.0f * v.x;
        float vy = 2.0f * v.y;
        float vz = 2.0f * v.z;
        float w2 = w * w - 0.5f;

        float dot2 = (x * vx + y * vy + z * vz);

        return Vec3(
            (vx * w2 - (y * vz - z * vy) * w + x * dot2), (vy * w2 - (z * vx - x * vz) * w + y * dot2),
            (vz * w2 - (x * vy - y * vx) * w + z * dot2)
        );
    }

    void SetIdentity()
    {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        w = 1.0f;
    }

    Vec3 ToEuler() const
    {
        // Roll (x-axis)
        float sinr_cosp = 2 * (w * x + y * z);
        float cosr_cosp = 1 - 2 * (x * x + y * y);
        float roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis)
        float sinp = 2 * (w * y - z * x);
        float pitch;
        if (std::abs(sinp) >= 1)
        {
            pitch = std::copysign(pi / 2, sinp); // use 90 degrees if out of range
        }
        else
        {
            pitch = std::asin(sinp);
        }

        // Yaw (z-axis)
        float siny_cosp = 2 * (w * z + x * y);
        float cosy_cosp = 1 - 2 * (y * y + z * z);
        float yaw = std::atan2(siny_cosp, cosy_cosp);

        return Vec3{ roll, pitch, yaw };
    }

    static inline Quat FromEuler(const Vec3& euler_angles)
    {
        float cr = std::cos(euler_angles.x * 0.5f);
        float sr = std::sin(euler_angles.x * 0.5f);
        float cp = std::cos(euler_angles.y * 0.5f);
        float sp = std::sin(euler_angles.y * 0.5f);
        float cy = std::cos(euler_angles.z * 0.5f);
        float sy = std::sin(euler_angles.z * 0.5f);

        Quat q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    std::string ToString() const
    {
        return FormatString("%.4f\t%.4f\t%.4f\t%.4f", x, y, z, w);
    }

    float x, y, z, w;
};

// Column major matrices

struct Mat2
{
    Vec2 ex, ey;

    Mat2() = default;

    constexpr Mat2(Identity)
        : Mat2(1)
    {
    }

    constexpr explicit Mat2(float v)
    {
        // clang-format off
        ex.x = v;   ey.x = 0;
        ex.y = 0;   ey.y = v;
        // clang-format on
    }

    constexpr explicit Mat2(const Vec2& v)
    {
        // clang-format off
        ex.x = v.x; ey.x = 0;
        ex.y = 0;   ey.y = v.y;
        // clang-format on
    }

    constexpr Mat2(const Vec2& c1, const Vec2& c2)
        : ex{ c1 }
        , ey{ c2 }
    {
    }

    Vec2& operator[](int32 i)
    {
        return (&ex)[i];
    }

    void SetIdentity()
    {
        // clang-format off
        ex.x = 1.0f;    ey.x = 0.0f;
        ex.y = 0.0f;    ey.y = 1.0f;
        // clang-format on
    }

    void SetZero()
    {
        // clang-format off
        ex.x = 0.0f;    ey.x = 0.0f;
        ex.y = 0.0f;    ey.y = 0.0f;
        // clang-format on
    }

    Mat2 GetTranspose()
    {
        Mat2 t;

        // clang-format off
        t.ex.x = ex.x;    t.ey.x = ex.y;
        t.ex.y = ey.x;    t.ey.y = ey.y;
        // clang-format on

        return t;
    }

    Mat2 GetInverse() const
    {
        Mat2 t;

        float det = ex.x * ey.y - ey.x * ex.y;
        if (det != 0.0f)
        {
            det = 1.0f / det;
        }

        t.ex.x = det * ey.y;
        t.ey.x = -det * ey.x;
        t.ex.y = -det * ex.y;
        t.ey.y = det * ex.x;

        return t;
    }

    float GetDeterminant() const
    {
        return ex.x * ey.y - ey.x * ex.y;
    }

    std::string ToString() const
    {
        return FormatString("%.4f\t%.4f\n%.4f\t%.4f", ex.x, ey.x, ex.y, ey.y);
    }
};

struct Mat3
{
    Vec3 ex, ey, ez;

    Mat3() = default;

    constexpr Mat3(Identity)
        : Mat3(1)
    {
    }

    constexpr explicit Mat3(float v)
    {
        // clang-format off
        ex.x = v;   ey.x = 0;   ez.x = 0;
        ex.y = 0;   ey.y = v;   ez.y = 0;
        ex.z = 0;   ey.z = 0;   ez.z = v;
        // clang-format on
    }

    constexpr explicit Mat3(const Vec3& v)
    {
        // clang-format off
        ex.x = v.x; ey.x = 0;   ez.x = 0;
        ex.y = 0;   ey.y = v.y; ez.y = 0;
        ex.z = 0;   ey.z = 0;   ez.z = v.z;
        // clang-format on
    }

    constexpr Mat3(const Vec3& c1, const Vec3& c2, const Vec3& c3)
        : ex{ c1 }
        , ey{ c2 }
        , ez{ c3 }
    {
    }

    Mat3(const Quat& q);

    Vec3& operator[](int32 i)
    {
        return (&ex)[i];
    }

    void SetIdentity()
    {
        // clang-format off
        ex.x = 1.0f;    ey.x = 0.0f;    ez.x = 0.0f;
        ex.y = 0.0f;    ey.y = 1.0f;    ez.y = 0.0f;
        ex.z = 0.0f;    ey.z = 0.0f;    ez.z = 1.0f;
        // clang-format on
    }

    void SetZero()
    {
        // clang-format off
        ex.x = 0.0f;    ey.x = 0.0f;    ez.x = 0.0f;
        ex.y = 0.0f;    ey.y = 0.0f;    ez.y = 0.0f;
        ex.z = 0.0f;    ey.z = 0.0f;    ez.z = 0.0f;
        // clang-format on
    }

    Mat3 GetTranspose()
    {
        Mat3 t;

        // clang-format off
        t.ex.x = ex.x;    t.ey.x = ex.y;    t.ez.x = ex.z;
        t.ex.y = ey.x;    t.ey.y = ey.y;    t.ez.y = ey.z;
        t.ex.z = ez.x;    t.ey.z = ez.y;    t.ez.z = ez.z;
        // clang-format on

        return t;
    }

    Mat3 GetInverse() const;
    Mat3 Scale(const Vec2& s);
    Mat3 Rotate(float z);
    Mat3 Translate(const Vec2& t);

    std::string ToString() const
    {
        return FormatString(
            "%.4f\t%.4f\t%.4f\n%.4f\t%.4f\t%.4f\n%.4f\t%.4f\t%.4f\n", ex.x, ey.x, ez.x, ex.y, ey.y, ez.y, ex.z, ey.z, ez.z
        );
    }
};

struct Mat4
{
    Vec4 ex, ey, ez, ew;

    Mat4() = default;

    constexpr Mat4(Identity)
        : Mat4(1.0f)
    {
    }

    constexpr Mat4(float v)
    {
        // clang-format off
        ex.x = v;       ey.x = 0.0f;    ez.x = 0.0f;    ew.x = 0.0f;
        ex.y = 0.0f;    ey.y = v;       ez.y = 0.0f;    ew.y = 0.0f;
        ex.z = 0.0f;    ey.z = 0.0f;    ez.z = v;       ew.z = 0.0f;
        ex.w = 0.0f;    ey.w = 0.0f;    ez.w = 0.0f;    ew.w = v;
        // clang-format on
    }

    constexpr explicit Mat4(const Vec4& v)
    {
        // clang-format off
        ex.x = v.x; ey.x = 0;   ez.x = 0;   ew.x = 0;
        ex.y = 0;   ey.y = v.y; ez.y = 0;   ew.y = 0;
        ex.z = 0;   ey.z = 0;   ez.z = v.z; ew.z = 0;
        ex.w = 0;   ey.w = 0;   ez.w = 0;   ew.w = v.w;
        // clang-format on
    }

    constexpr Mat4(const Vec4& c1, const Vec4& c2, const Vec4& c3, const Vec4& c4)
        : ex{ c1 }
        , ey{ c2 }
        , ez{ c3 }
        , ew{ c4 }
    {
    }

    constexpr Mat4(const Mat3& r, const Vec3& p)
        : ex{ r.ex, 0 }
        , ey{ r.ey, 0 }
        , ez{ r.ez, 0 }
        , ew{ p, 1 }
    {
    }

    Mat4(const Transform& t);

    Vec4& operator[](int32 i)
    {
        return (&ex)[i];
    }

    void SetIdentity()
    {
        // clang-format off
        ex.x = 1.0f;    ey.x = 0.0f;    ez.x = 0.0f;    ew.x = 0.0f;
        ex.y = 0.0f;    ey.y = 1.0f;    ez.y = 0.0f;    ew.y = 0.0f;
        ex.z = 0.0f;    ey.z = 0.0f;    ez.z = 1.0f;    ew.z = 0.0f;
        ex.w = 0.0f;    ey.w = 0.0f;    ez.w = 0.0f;    ew.w = 1.0f;
        // clang-format on
    }

    void SetZero()
    {
        // clang-format off
        ex.x = 0.0f;    ey.x = 0.0f;    ez.x = 0.0f;    ew.x = 0.0f;
        ex.y = 0.0f;    ey.y = 0.0f;    ez.y = 0.0f;    ew.y = 0.0f;
        ex.z = 0.0f;    ey.z = 0.0f;    ez.z = 0.0f;    ew.z = 0.0f;
        ex.w = 0.0f;    ey.w = 0.0f;    ez.w = 0.0f;    ew.w = 0.0f;
        // clang-format on
    }

    Mat4 GetTranspose()
    {
        Mat4 t;

        // clang-format off
        t.ex.x = ex.x;    t.ey.x = ex.y;    t.ez.x = ex.z;    t.ew.x = ex.w;
        t.ex.y = ey.x;    t.ey.y = ey.y;    t.ez.y = ey.z;    t.ew.y = ey.w;
        t.ex.z = ez.x;    t.ey.z = ez.y;    t.ez.z = ez.z;    t.ew.z = ez.w;
        t.ex.w = ew.x;    t.ey.w = ew.y;    t.ez.w = ew.z;    t.ew.w = ew.w;
        // clang-format on

        return t;
    }

    Mat4 GetInverse();
    Mat4 Scale(const Vec3& s);
    Mat4 Rotate(const Vec3& r);
    Mat4 Translate(const Vec3& t);

    static Mat4 Orth(float left, float right, float bottom, float top, float zNear, float zFar);
    static Mat4 Perspective(float fovY, float aspect, float zNear, float zFar);

    std::string ToString() const
    {
        return FormatString(
            "%.4f\t%.4f\t%.4f\t%.4f\n%.4f\t%.4f\t%.4f\t%.4f\n%.4f\t%.4f\t%.4f\t%.4f\n%.4f\t%.4f\t%.4f\t%.4f\n", ex.x, ey.x, ez.x,
            ew.x, ex.y, ey.y, ez.y, ew.y, ex.z, ey.z, ez.z, ew.z, ex.w, ey.w, ez.w, ew.w
        );
    }
};

// Describes 2d orientation
struct Rotation
{
    // sine, cosine
    float s, c;

    Rotation() = default;

    Rotation(Identity)
        : Rotation(0.0f)
    {
    }

    explicit Rotation(float angle)
    {
        s = sinf(angle);
        c = cosf(angle);
    }

    void operator=(float angle)
    {
        s = sinf(angle);
        c = cosf(angle);
    }

    void SetIdentity()
    {
        s = 0.0f;
        c = 1.0f;
    }

    float GetAngle() const
    {
        return atan2f(s, c);
    }
};

struct Transform
{
    Transform() = default;

    Transform(Identity)
        : position{ 0.0f }
        , rotation{ identity }
    {
    }

    Transform(const Vec2& position, const Rotation& rotation)
        : position{ position }
        , rotation{ rotation }
    {
    }

    Transform(const Vec2& position, float angle)
        : position{ position }
        , rotation{ angle }
    {
    }

    void Set(const Vec2& _position, const Rotation& _rotation)
    {
        position = _position;
        rotation = _rotation;
    }

    void SetIdentity()
    {
        position.SetZero();
        rotation.SetIdentity();
    }

    Vec2 position;
    Rotation rotation;
};

// Describes the swept motion of rigid body
struct Sweep
{
    Sweep() = default;

    Sweep(Identity)
        : localCenter{ 0.0f }
        , c0{ 0.0f }
        , c{ 0.0f }
        , a0{ 0.0f }
        , a{ 0.0f }
        , alpha0{ 0.0f }
    {
    }

    // Get the interpolated transform at time beta, where alpha0 <= beta <= alpha
    void GetTransform(float beta, Transform* transform) const;

    // Advance the sweep forward, yielding a new initial state.
    void Advance(float alpha);

    // Normalize an angle in radians to be between -pi and pi
    void Normalize();

    Vec2 localCenter; // Center of mass relative to the local origin
    Vec2 c0, c;       // World space position relative to the center of mass
    float a0, a;      // World space rotation

    float alpha0;
};

// Vec2 functions begin

inline float Dot(const Vec2& a, const Vec2& b)
{
    return a.x * b.x + a.y * b.y;
}

inline float Cross(const Vec2& a, const Vec2& b)
{
    return a.x * b.y - a.y * b.x;
}

inline Vec2 Cross(float s, const Vec2& v)
{
    return Vec2{ -s * v.y, s * v.x };
}

inline Vec2 Cross(const Vec2& v, float s)
{
    return Vec2{ s * v.y, -s * v.x };
}

inline Vec2 operator+(const Vec2& a, const Vec2& b)
{
    return Vec2{ a.x + b.x, a.y + b.y };
}

inline Vec2 operator-(const Vec2& a, const Vec2& b)
{
    return Vec2{ a.x - b.x, a.y - b.y };
}

inline Vec2 operator*(const Vec2& v, float s)
{
    return Vec2{ v.x * s, v.y * s };
}

inline Vec2 operator*(float s, const Vec2& v)
{
    return Vec2{ v.x * s, v.y * s };
}

inline Vec2 operator/(const Vec2& v, float s)
{
    return v * (1.0f / s);
}

inline bool operator==(const Vec2& a, const Vec2& b)
{
    return a.x == b.x && a.y == b.y;
}

inline bool operator!=(const Vec2& a, const Vec2& b)
{
    return a.x != b.x || a.y != b.y;
}

inline float Dist(const Vec2& a, const Vec2& b)
{
    return (a - b).Length();
}

inline float Dist2(const Vec2& a, const Vec2& b)
{
    return (a - b).Length2();
}

inline float Length(const Vec2& v)
{
    return v.Length();
}

inline float Length2(const Vec2& v)
{
    return v.Length2();
}

// Vec2 functions end

// Vec3 functions begin

inline float Dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 Cross(const Vec3& a, const Vec3& b)
{
    return Vec3{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

inline Vec3 operator+(const Vec3& a, const Vec3& b)
{
    return Vec3{ a.x + b.x, a.y + b.y, a.z + b.z };
}

inline Vec3 operator-(const Vec3& a, const Vec3& b)
{
    return Vec3{ a.x - b.x, a.y - b.y, a.z - b.z };
}

inline Vec3 operator*(const Vec3& v, float s)
{
    return Vec3{ v.x * s, v.y * s, v.z * s };
}

inline Vec3 operator*(float s, const Vec3& v)
{
    return Vec3{ v.x * s, v.y * s, v.z * s };
}

inline Vec3 operator/(const Vec3& v, float s)
{
    return v * (1.0f / s);
}

inline bool operator==(const Vec3& a, const Vec3& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

inline bool operator!=(const Vec3& a, const Vec3& b)
{
    return a.x != b.x || a.y != b.y || a.z != b.z;
}

inline float Dist(const Vec3& a, const Vec3& b)
{
    return (b - a).Length();
}

inline float Dist2(const Vec3& a, const Vec3& b)
{
    return (b - a).Length2();
}

// Vec3 functions end

// Vec4 functions begin

inline float Dot(const Vec4& a, const Vec4& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline Vec4 operator+(const Vec4& a, const Vec4& b)
{
    return Vec4{ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w };
}

inline Vec4 operator-(const Vec4& a, const Vec4& b)
{
    return Vec4{ a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w };
}

inline bool operator==(const Vec4& a, const Vec4& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

inline bool operator!=(const Vec4& a, const Vec4& b)
{
    return a.x != b.x || a.y != b.y || a.z != b.z || a.w != b.w;
}

inline Vec4 operator*(const Vec4& v, float s)
{
    return Vec4{ v.x * s, v.y * s, v.z * s, v.w * s };
}

inline Vec4 operator*(float s, const Vec4& v)
{
    return Vec4{ v.x * s, v.y * s, v.z * s, v.w * s };
}

inline Vec4 operator/(const Vec4& v, float s)
{
    return v * (1.0f / s);
}

// Vec4 functions end

template <typename T>
inline T Normalize(const T& v)
{
    float invLength = 1.0f / v.Length();
    return v * invLength;
}

template <typename T>
inline T NormalizeSafe(const T& v)
{
    float length = v.Length();
    if (length < epsilon)
    {
        return T::zero;
    }

    float invLength = 1.0f / length;
    return v * invLength;
}

// Mat2 functions begin

inline Mat2 operator+(const Mat2& a, const Mat2& b)
{
    return Mat2{ a.ex + b.ex, a.ey + b.ey };
}

// M * V
inline Vec2 Mul(const Mat2& m, const Vec2& v)
{
    return Vec2{
        m.ex.x * v.x + m.ey.x * v.y,
        m.ex.y * v.x + m.ey.y * v.y,
    };
}

inline Vec2 operator*(const Mat2& m, const Vec2& v)
{
    return Vec2{
        m.ex.x * v.x + m.ey.x * v.y,
        m.ex.y * v.x + m.ey.y * v.y,
    };
}

// M^T * V
inline Vec2 MulT(const Mat2& m, const Vec2& v)
{
    return Vec2{ Dot(m.ex, v), Dot(m.ey, v) };
}

// A * B
inline Mat2 Mul(const Mat2& a, const Mat2& b)
{
    return Mat2{ a * b.ex, a * b.ey };
}

inline Mat2 operator*(const Mat2& a, const Mat2& b)
{
    return Mat2{ a * b.ex, a * b.ey };
}

// A^T * B
inline Mat2 MulT(const Mat2& a, const Mat2& b)
{
    Vec2 c1{ Dot(a.ex, b.ex), Dot(a.ey, b.ex) };
    Vec2 c2{ Dot(a.ex, b.ey), Dot(a.ey, b.ey) };

    return Mat2{ c1, c2 };
}

// Mat2 functions end

// Mat3 functions begin

// M * V
inline Vec3 Mul(const Mat3& m, const Vec3& v)
{
    return Vec3{
        m.ex.x * v.x + m.ey.x * v.y + m.ez.x * v.z,
        m.ex.y * v.x + m.ey.y * v.y + m.ez.y * v.z,
        m.ex.z * v.x + m.ey.z * v.y + m.ez.z * v.z,
    };
}

inline Vec3 operator*(const Mat3& m, const Vec3& v)
{
    return Vec3{
        m.ex.x * v.x + m.ey.x * v.y + m.ez.x * v.z,
        m.ex.y * v.x + m.ey.y * v.y + m.ez.y * v.z,
        m.ex.z * v.x + m.ey.z * v.y + m.ez.z * v.z,
    };
}

// M^T * V
inline Vec3 MulT(const Mat3& m, const Vec3& v)
{
    return Vec3{ Dot(m.ex, v), Dot(m.ey, v), Dot(m.ez, v) };
}

// A * B
inline Mat3 Mul(const Mat3& a, const Mat3& b)
{
    return Mat3{ a * b.ex, a * b.ey, a * b.ez };
}

inline Mat3 operator*(const Mat3& a, const Mat3& b)
{
    return Mat3{ a * b.ex, a * b.ey, a * b.ez };
}

// A^T * B
inline Mat3 MulT(const Mat3& a, const Mat3& b)
{
    Vec3 c1{ Dot(a.ex, b.ex), Dot(a.ey, b.ex), Dot(a.ez, b.ex) };
    Vec3 c2{ Dot(a.ex, b.ey), Dot(a.ey, b.ey), Dot(a.ez, b.ey) };
    Vec3 c3{ Dot(a.ex, b.ez), Dot(a.ey, b.ez), Dot(a.ez, b.ez) };

    return Mat3{ c1, c2, c3 };
}

// Mat3 functions end

// Mat4 functions begin

// M * V
inline Vec4 Mul(const Mat4& m, const Vec4& v)
{
    return Vec4{
        m.ex.x * v.x + m.ey.x * v.y + m.ez.x * v.z + m.ew.x * v.w,
        m.ex.y * v.x + m.ey.y * v.y + m.ez.y * v.z + m.ew.y * v.w,
        m.ex.z * v.x + m.ey.z * v.y + m.ez.z * v.z + m.ew.z * v.w,
        m.ex.w * v.x + m.ey.w * v.y + m.ez.w * v.z + m.ew.w * v.w,
    };
}

inline Vec4 operator*(const Mat4& m, const Vec4& v)
{
    return Vec4{
        m.ex.x * v.x + m.ey.x * v.y + m.ez.x * v.z + m.ew.x * v.w,
        m.ex.y * v.x + m.ey.y * v.y + m.ez.y * v.z + m.ew.y * v.w,
        m.ex.z * v.x + m.ey.z * v.y + m.ez.z * v.z + m.ew.z * v.w,
        m.ex.w * v.x + m.ey.w * v.y + m.ez.w * v.z + m.ew.w * v.w,
    };
}

// M^T * V
inline Vec4 MulT(const Mat4& m, const Vec4& v)
{
    return Vec4{ Dot(m.ex, v), Dot(m.ey, v), Dot(m.ez, v), Dot(m.ew, v) };
}

// A * B
inline Mat4 Mul(const Mat4& a, const Mat4& b)
{
    return Mat4{ a * b.ex, a * b.ey, a * b.ez, a * b.ew };
}

inline Mat4 operator*(const Mat4& a, const Mat4& b)
{
    return Mat4{ a * b.ex, a * b.ey, a * b.ez, a * b.ew };
}

// A^T * B
inline Mat4 MulT(const Mat4& a, const Mat4& b)
{
    Vec4 c1{ Dot(a.ex, b.ex), Dot(a.ey, b.ex), Dot(a.ez, b.ex), Dot(a.ew, b.ex) };
    Vec4 c2{ Dot(a.ex, b.ey), Dot(a.ey, b.ey), Dot(a.ez, b.ey), Dot(a.ew, b.ey) };
    Vec4 c3{ Dot(a.ex, b.ez), Dot(a.ey, b.ez), Dot(a.ez, b.ez), Dot(a.ew, b.ez) };
    Vec4 c4{ Dot(a.ex, b.ew), Dot(a.ey, b.ew), Dot(a.ez, b.ew), Dot(a.ew, b.ew) };

    return Mat4{ c1, c2, c3, c4 };
}

inline Mat4 Mat4::Orth(float left, float right, float bottom, float top, float z_near, float z_far)
{
    Mat4 t{ identity };

    // Scale
    t.ex.x = 2 / (right - left);
    t.ey.y = 2 / (top - bottom);
    t.ez.z = 2 / (z_far - z_near);

    // Translation
    t.ew.x = -(right + left) / (right - left);
    t.ew.y = -(top + bottom) / (top - bottom);
    t.ew.z = -(z_far + z_near) / (z_far - z_near);

    return t;
}

inline Mat4 Mat4::Perspective(float vertical_fov, float aspect_ratio, float z_near, float z_far)
{
    Mat4 t{ identity };

    float tan_half_fov = std::tan(vertical_fov / 2);

    // Scale
    t.ex.x = 1 / (aspect_ratio * tan_half_fov);
    t.ey.y = 1 / tan_half_fov;
    t.ez.z = -(z_far + z_near) / (z_far - z_near);
    t.ez.w = -1; // Needed for perspective division

    // Translation (for z-axis)
    t.ew.z = -(2 * z_far * z_near) / (z_far - z_near);

    // No translation in x or y
    t.ew.w = 0;

    return t;
}

// Mat4 functions end

// Rotate a vector
inline Vec2 Mul(const Rotation& r, const Vec2& v)
{
    return Vec2{ r.c * v.x - r.s * v.y, r.s * v.x + r.c * v.y };
}

// Inverse rotate a vector
inline Vec2 MulT(const Rotation& r, const Vec2& v)
{
    return Vec2{ r.c * v.x + r.s * v.y, -r.s * v.x + r.c * v.y };
}

// Transform a vector: simplified matrix multiplication
inline Vec2 Mul(const Transform& t, const Vec2& v)
{
    float x = (t.rotation.c * v.x - t.rotation.s * v.y) + t.position.x;
    float y = (t.rotation.s * v.x + t.rotation.c * v.y) + t.position.y;

    return Vec2{ x, y };
}

// Inverse transform a vector
inline Vec2 MulT(const Transform& t, const Vec2& v)
{
    float px = v.x - t.position.x;
    float py = v.y - t.position.y;
    float x = (t.rotation.c * px + t.rotation.s * py);
    float y = (-t.rotation.s * px + t.rotation.c * py);

    return Vec2{ x, y };
}

// Generals

template <typename T>
inline T Abs(T a)
{
    return a > T(0) ? a : -a;
}

inline Vec2 Abs(const Vec2& a)
{
    return Vec2(Abs(a.x), Abs(a.y));
}

inline Mat2 Abs(const Mat2& A)
{
    return Mat2(Abs(A.ex), Abs(A.ey));
}

inline float Min(float a, float b)
{
    return fminf(a, b);
}

template <typename T>
inline T Min(T a, T b)
{
    return a < b ? a : b;
}

inline Vec2 Min(const Vec2& a, const Vec2& b)
{
    return Vec2(Min(a.x, b.x), Min(a.y, b.y));
}

inline Vec3 Min(const Vec3& a, const Vec3& b)
{
    return Vec3(Min(a.x, b.x), Min(a.y, b.y), Min(a.z, b.z));
}

inline float Max(float a, float b)
{
    return fmaxf(a, b);
}

template <typename T>
inline T Max(T a, T b)
{
    return a > b ? a : b;
}

inline Vec2 Max(const Vec2& a, const Vec2& b)
{
    return Vec2(Max(a.x, b.x), Max(a.y, b.y));
}

inline Vec3 Max(const Vec3& a, const Vec3& b)
{
    return Vec3(Max(a.x, b.x), Max(a.y, b.y), Max(a.z, b.z));
}

inline Vec2 Clamp(const Vec2& a, const Vec2& _min, const Vec2& _max)
{
    return Max(_min, Min(a, _max));
}

template <typename T>
inline T Clamp(T v, T _min, T _max)
{
    return Max(_min, Min(v, _max));
}

inline float Floor(float s)
{
    return floorf(s);
}

inline float Sqrt(float s)
{
    return sqrtf(s);
}

inline float Log(float s)
{
    return logf(s);
}

inline float Trunc(float s)
{
    return truncf(s);
}

inline float Cos(float s)
{
    return cosf(s);
}

inline float Sin(float s)
{
    return sinf(s);
}

inline float Tan(float s)
{
    return tanf(s);
}

inline float Acos(float s)
{
    return acosf(s);
}

inline float Asin(float s)
{
    return asinf(s);
}

inline float Atan(float s)
{
    return atanf(s);
}

inline float Atan2(float y, float x)
{
    return atan2f(y, x);
}

inline float Round(float s)
{
    return roundf(s);
}

inline float DegToRad(float deg)
{
    return deg * pi / 180.0f;
}

inline float RadToDeg(float rad)
{
    return rad * 180.0f / pi;
}

inline float Lerp(float left, float right, float per)
{
    return left + (right - left) * per;
}

template <typename T>
inline T Lerp(const T& a, const T& b, float t)
{
    return a * (1.0f - t) + b * t;
}

inline float Map(float v, float left, float right, float min, float max)
{
    float per = (v - left) / (right - left);

    return Lerp(min, max, per);
}

template <typename T>
inline T Slerp(const T& start, const T& end, float percent)
{
    float dot = Clamp(Dot(start, end), -1.0f, 1.0f);
    float angle = acosf(dot) * percent;

    T rv = end - start * dot;
    rv.Normalize();

    return start * Cos(angle) + rv * Sin(angle);
}

template <typename T>
inline T Reflect(const T& v, const T& n)
{
    return v - 2.0f * Dot(v, n) * n;
}

inline float AngleBetween(const Vec2& a, const Vec2& b)
{
    float sine = Cross(a, b);
    float cosine = Dot(a, b);

    return Atan2(sine, cosine);
}

inline Vec2 PolarToCart(float theta, float r)
{
    float x = Cos(theta);
    float y = Sin(theta);

    return Vec2{ x * r, y * r };
}

inline Vec3 PolarToCart(float phi, float theta, float r)
{
    float x = Sin(phi) * Cos(theta);
    float y = Sin(phi) * Sin(theta);
    float z = Cos(phi);

    return Vec3{ x * r, y * r, z * r };
}

inline void Sweep::GetTransform(float beta, Transform* transform) const
{
    transform->position = (1.0f - beta) * c0 + beta * c;
    float angle = (1.0f - beta) * a0 + beta * a;
    transform->rotation = angle;

    // Shift to origin
    transform->position -= Mul(transform->rotation, localCenter);
}

inline void Sweep::Advance(float alpha)
{
    // alpha0 < alpha < 1.0f
    assert(alpha0 < 1.0f);
    float beta = (alpha - alpha0) / (1.0f - alpha0);
    c0 += beta * (c - c0);
    a0 += beta * (a - a0);
    alpha0 = alpha;
}

inline void Sweep::Normalize()
{
    float pi2 = 2.0f * pi;
    float d = pi2 * Floor(a0 / pi2);
    a0 -= d;
    a -= d;
}

} // namespace muli
