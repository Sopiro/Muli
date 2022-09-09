// Simple linear math library
// Highly inspired by Box2d, glm math codes

#pragma once

#include <limits>
#include <math.h>
#include <stdint.h>

#define SPE_PI 3.14159265359f

namespace spe
{

struct Vec2
{
    float x, y;

    Vec2() = default;

    constexpr Vec2(float _v)
        : x{ _v }
        , y{ _v }
    {
    }

    constexpr Vec2(float _x, float _y)
        : x{ _x }
        , y{ _y }
    {
    }

    void SetZero()
    {
        x = 0.0f;
        y = 0.0f;
    }

    void Set(float _x, float _y)
    {
        x = _x;
        y = _y;
    }

    float& operator[](uint32_t i)
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

    // Optimized to not check length == 0
    float Normalize()
    {
        float length = Length();
        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;

        return length;
    }

    // Optimized to not check length == 0
    Vec2 Normalized() const
    {
        float invLength = 1.0f / Length();

        return Vec2{ x * invLength, y * invLength };
    }
};

struct Vec3
{
    float x, y, z;

    Vec3() = default;

    constexpr Vec3(float _v)
        : x{ _v }
        , y{ _v }
        , z{ _v }
    {
    }

    constexpr Vec3(float _x, float _y, float _z)
        : x{ _x }
        , y{ _y }
        , z{ _z }
    {
    }

    void SetZero()
    {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }

    void Set(float _x, float _y, float _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }

    float& operator[](uint32_t i)
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

    void Normalize()
    {
        float length = Length();
        if (length < FLT_EPSILON)
        {
            return;
        }

        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;
        z *= invLength;
    }

    Vec3 Normalized() const
    {
        float length = Length();
        if (length < FLT_EPSILON)
        {
            return Vec3{ 0.0f, 0.0f, 0.0f };
        }

        float invLength = 1.0f / length;

        return Vec3{ x * invLength, y * invLength, z * invLength };
    }
};

struct Vec4
{
    float x, y, z, w;

    Vec4() = default;

    constexpr Vec4(float _v, float _w)
        : x{ _v }
        , y{ _v }
        , z{ _v }
        , w{ _w }
    {
    }

    constexpr Vec4(float _x, float _y, float _z, float _w)
        : x{ _x }
        , y{ _y }
        , z{ _z }
        , w{ _w }
    {
    }

    float& operator[](uint32_t i)
    {
        return (&x)[i];
    }
};

// Column major matrices

struct Mat2
{
    Vec2 ex, ey;

    Mat2() = default;

    Mat2(float v)
    {
        // clang-format off
        ex.x = v;       ey.x = 0.0f;
        ex.y = 0.0f;    ey.y = v;
        // clang-format on
    }

    constexpr Mat2(const Vec2& c1, const Vec2& c2)
        : ex{ c1 }
        , ey{ c2 }
    {
    }

    Vec2& operator[](uint32_t i)
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
};

struct Mat3
{
    Vec3 ex, ey, ez;

    Mat3() = default;

    Mat3(float v)
    {
        // clang-format off
        ex.x = v;       ey.x = 0.0f;    ez.x = 0.0f;
        ex.y = 0.0f;    ey.y = v;       ez.y = 0.0f;
        ex.z = 0.0f;    ey.z = 0.0f;    ez.z = v;
        // clang-format on
    }

    constexpr Mat3(const Vec3& c1, const Vec3& c2, const Vec3& c3)
        : ex{ c1 }
        , ey{ c2 }
        , ez{ c3 }
    {
    }

    Vec3& operator[](uint32_t i)
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

    Mat3 GetInverse() const
    {
        Mat3 t;

        float det = ex.x * (ey.y * ez.z - ey.z * ez.y) - ey.x * (ex.y * ez.z - ez.y * ex.z) + ez.x * (ex.y * ey.z - ey.y * ex.z);

        if (det != 0)
        {
            det = 1.0f / det;
        }

        t.ex.x = (ey.y * ez.z - ey.z * ez.y) * det;
        t.ey.x = (ez.x * ey.z - ey.x * ez.z) * det;
        t.ez.x = (ey.x * ez.y - ez.x * ey.y) * det;
        t.ex.y = (ez.y * ex.z - ex.y * ez.z) * det;
        t.ey.y = (ex.x * ez.z - ez.x * ex.z) * det;
        t.ez.y = (ex.y * ez.x - ex.x * ez.y) * det;
        t.ex.z = (ex.y * ey.z - ex.z * ey.y) * det;
        t.ey.z = (ex.z * ey.x - ex.x * ey.z) * det;
        t.ez.z = (ex.x * ey.y - ex.y * ey.x) * det;

        return t;
    }
};

struct Mat4
{
    Vec4 ex, ey, ez, ew;

    Mat4() = default;

    Mat4(float _v)
    {
        // clang-format off
        ex.x = _v;      ey.x = 0.0f;    ez.x = 0.0f;    ew.x = 0.0f;
        ex.y = 0.0f;    ey.y = _v;      ez.y = 0.0f;    ew.y = 0.0f;
        ex.z = 0.0f;    ey.z = 0.0f;    ez.z = _v;      ew.z = 0.0f;
        ex.w = 0.0f;    ey.w = 0.0f;    ez.w = 0.0f;    ew.w = _v;
        // clang-format on
    }

    constexpr Mat4(const Vec4& c1, const Vec4& c2, const Vec4& c3, const Vec4& c4)
        : ex{ c1 }
        , ey{ c2 }
        , ez{ c3 }
        , ew{ c4 }
    {
    }

    Vec4& operator[](uint32_t i)
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

    Mat4 Scale(float x, float y, float z);

    Mat4 Rotate(float x, float y, float z);

    Mat4 Translate(float x, float y, float z);

    Mat4 Translate(const Vec3& v);
};

struct Rotation
{
    float angle = 0.0f;

    // sine, cosine
    float s = 0.0f;
    float c = 1.0f;

    Rotation() = default;

    explicit Rotation(float _angle)
    {
        angle = _angle;
        s = sinf(_angle);
        c = cosf(_angle);
    }

    void operator+=(float _angle)
    {
        angle += _angle;
        s = sinf(angle);
        c = cosf(angle);
    }

    void operator-=(float _angle)
    {
        angle -= _angle;
        s = sinf(angle);
        c = cosf(angle);
    }

    void operator=(float _angle)
    {
        angle = _angle;
        s = sinf(angle);
        c = cosf(angle);
    }

    void SetIdentity()
    {
        angle = 0.0f;
        s = 0.0f;
        c = 1.0f;
    }

    float GetAngle() const
    {
        return angle;
    }
};

struct Transform
{
    Vec2 position{ 0.0f, 0.0f };
    Rotation rotation{};

    Transform() = default;

    Transform(const Vec2& _position, const Rotation& _rotation)
        : position{ _position }
        , rotation{ _rotation }
    {
    }

    void Set(const Vec2& _position, const Rotation& _rotation)
    {
        position = _position;
        rotation = _rotation;
    }

    void SetIdentity()
    {
        position.x = 0.0f;
        position.y = 0.0f;
        rotation.SetIdentity();
    }
};

// Vec2 functions begin

inline float dot(const Vec2& a, const Vec2& b)
{
    return a.x * b.x + a.y * b.y;
}

inline float cross(const Vec2& a, const Vec2& b)
{
    return a.x * b.y - a.y * b.x;
}

inline Vec2 cross(float s, const Vec2& v)
{
    return Vec2{ -s * v.y, s * v.x };
}

inline Vec2 cross(const Vec2& v, float s)
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

inline float dist(const Vec2& a, const Vec2& b)
{
    return (a - b).Length();
}

inline float dist2(const Vec2& a, const Vec2& b)
{
    return (a - b).Length2();
}

inline float length(const Vec2& v)
{
    return v.Length();
}

inline float length2(const Vec2& v)
{
    return v.Length2();
}

inline Vec2 normalize(const Vec2& v)
{
    return v.Normalized();
}

// Vec2 functions end

// Vec3 functions begin

inline float dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b)
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

inline float dist(const Vec3& a, const Vec3& b)
{
    return (b - a).Length();
}

inline float dist2(const Vec3& a, const Vec3& b)
{
    return (b - a).Length2();
}

// Vec3 functions end

// Vec4 functions begin

inline float dot(const Vec4& a, const Vec4& b)
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

// Vec4 functions end

// Mat2 functions begin

inline Mat2 operator+(const Mat2& a, const Mat2& b)
{
    return Mat2{ a.ex + b.ex, a.ey + b.ey };
}

// M * V
inline Vec2 mul(const Mat2& m, const Vec2& v)
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
inline Vec2 mul_t(const Mat2& m, const Vec2& v)
{
    return Vec2{ dot(m.ex, v), dot(m.ey, v) };
}

// A * B
inline Mat2 mul(const Mat2& a, const Mat2& b)
{
    return Mat2{ a * b.ex, a * b.ey };
}

inline Mat2 operator*(const Mat2& a, const Mat2& b)
{
    return Mat2{ a * b.ex, a * b.ey };
}

// A^T * B
inline Mat2 mul_t(const Mat2& a, const Mat2& b)
{
    Vec2 c1{ dot(a.ex, b.ex), dot(a.ey, b.ex) };
    Vec2 c2{ dot(a.ex, b.ey), dot(a.ey, b.ey) };

    return Mat2{ c1, c2 };
}

// Mat2 functions end

// Mat3 functions begin

// M * V
inline Vec3 mul(const Mat3& m, const Vec3& v)
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
inline Vec3 mul_t(const Mat3& m, const Vec3& v)
{
    return Vec3{ dot(m.ex, v), dot(m.ey, v), dot(m.ez, v) };
}

// A * B
inline Mat3 mul(const Mat3& a, const Mat3& b)
{
    return Mat3{ a * b.ex, a * b.ey, a * b.ez };
}

inline Mat3 operator*(const Mat3& a, const Mat3& b)
{
    return Mat3{ a * b.ex, a * b.ey, a * b.ez };
}

// A^T * B
inline Mat3 mul_t(const Mat3& a, const Mat3& b)
{
    Vec3 c1{ dot(a.ex, b.ex), dot(a.ey, b.ex), dot(a.ez, b.ex) };
    Vec3 c2{ dot(a.ex, b.ey), dot(a.ey, b.ey), dot(a.ez, b.ey) };
    Vec3 c3{ dot(a.ex, b.ez), dot(a.ey, b.ez), dot(a.ez, b.ez) };

    return Mat3{ c1, c2, c3 };
}

// Mat3 functions end

// Mat4 functions begin

// M * V
inline Vec4 mul(const Mat4& m, const Vec4& v)
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
inline Vec4 mul_t(const Mat4& m, const Vec4& v)
{
    return Vec4{ dot(m.ex, v), dot(m.ey, v), dot(m.ez, v), dot(m.ew, v) };
}

// A * B
inline Mat4 mul(const Mat4& a, const Mat4& b)
{
    return Mat4{ a * b.ex, a * b.ey, a * b.ez, a * b.ew };
}

inline Mat4 operator*(const Mat4& a, const Mat4& b)
{
    return Mat4{ a * b.ex, a * b.ey, a * b.ez, a * b.ew };
}

// A^T * B
inline Mat4 mul_t(const Mat4& a, const Mat4& b)
{
    Vec4 c1{ dot(a.ex, b.ex), dot(a.ey, b.ex), dot(a.ez, b.ex), dot(a.ew, b.ex) };
    Vec4 c2{ dot(a.ex, b.ey), dot(a.ey, b.ey), dot(a.ez, b.ey), dot(a.ew, b.ey) };
    Vec4 c3{ dot(a.ex, b.ez), dot(a.ey, b.ez), dot(a.ez, b.ez), dot(a.ew, b.ez) };
    Vec4 c4{ dot(a.ex, b.ew), dot(a.ey, b.ew), dot(a.ez, b.ew), dot(a.ew, b.ew) };

    return Mat4{ c1, c2, c3, c4 };
}

inline Mat4 orth(float left, float right, float bottom, float top, float zNear, float zFar)
{
    Mat4 t{ 1.0f };

    // Scale
    t.ex.x = 2.0f / (right - left);
    t.ey.y = 2.0f / (top - bottom);
    t.ez.z = 2.0f / (zFar - zNear);

    // Translation
    t.ew.x = -(right + left) / (right - left);
    t.ew.y = -(top + bottom) / (top - bottom);
    t.ew.z = -(zFar + zNear) / (zFar - zNear);

    return t;
}

// Mat4 functions end

// Rotate a vector
inline Vec2 mul(const Rotation& r, const Vec2& v)
{
    return Vec2{ r.c * v.x - r.s * v.y, r.s * v.x + r.c * v.y };
}

inline Vec2 operator*(const Rotation& r, const Vec2& v)
{
    return Vec2{ r.c * v.x - r.s * v.y, r.s * v.x + r.c * v.y };
}

// Inverse rotate a vector
inline Vec2 mul_t(const Rotation& r, const Vec2& v)
{
    return Vec2{ r.c * v.x + r.s * v.y, -r.s * v.x + r.c * v.y };
}

// Transform a vector: simplified matrix multiplication
inline Vec2 mul(const Transform& t, const Vec2& v)
{
    float x = (t.rotation.c * v.x - t.rotation.s * v.y) + t.position.x;
    float y = (t.rotation.s * v.x + t.rotation.c * v.y) + t.position.y;

    return Vec2{ x, y };
}

inline Vec2 operator*(const Transform& t, const Vec2& v)
{
    float x = (t.rotation.c * v.x - t.rotation.s * v.y) + t.position.x;
    float y = (t.rotation.s * v.x + t.rotation.c * v.y) + t.position.y;

    return Vec2{ x, y };
}

// Inverse transform a vector
inline Vec2 mul_t(const Transform& t, const Vec2& v)
{
    float px = v.x - t.position.x;
    float py = v.y - t.position.y;
    float x = (t.rotation.c * px + t.rotation.s * py);
    float y = (-t.rotation.s * px + t.rotation.c * py);

    return Vec2{ x, y };
}

// Generals

template <typename T>
inline T abs(T a)
{
    return a > T(0) ? a : -a;
}

inline Vec2 abs(const Vec2& a)
{
    return Vec2(abs(a.x), abs(a.y));
}

inline Mat2 abs(const Mat2& A)
{
    return Mat2(abs(A.ex), abs(A.ey));
}

template <typename T>
inline T min(T a, T b)
{
    return a < b ? a : b;
}

inline Vec2 min(const Vec2& a, const Vec2& b)
{
    return Vec2(min(a.x, b.x), min(a.y, b.y));
}

template <typename T>
inline T max(T a, T b)
{
    return a > b ? a : b;
}

inline Vec2 max(const Vec2& a, const Vec2& b)
{
    return Vec2(max(a.x, b.x), max(a.y, b.y));
}

template <typename T>
inline T clamp(T v, T _min, T _max)
{
    return max(_min, min(v, _max));
}

inline Vec2 clamp(const Vec2& a, const Vec2& _min, const Vec2& _max)
{
    return max(_min, min(a, _max));
}

inline float floor(float s)
{
    return floorf(s);
}

inline float sqrt(float s)
{
    return sqrtf(s);
}

inline float log(float s)
{
    return logf(s);
}

inline float trunc(float s)
{
    return truncf(s);
}

inline float cos(float s)
{
    return cosf(s);
}

inline float sin(float s)
{
    return sinf(s);
}

} // namespace spe
