#include "muli/math.h"

namespace muli
{

// https://math.stackexchange.com/questions/893984/conversion-of-rotation-matrix-to-quaternion
Quat::Quat(const Mat3& m)
{
    if (m.ez.z < float(0))
    {
        if (m.ex.x > m.ey.y)
        {
            float t = 1.0f + m.ex.x - m.ey.y - m.ez.z;
            *this = Quat(t, m.ex.y + m.ey.x, m.ez.x + m.ex.z, m.ey.z - m.ez.y) * (0.5f / sqrtf(t));
        }
        else
        {
            float t = 1.0f - m.ex.x + m.ey.y - m.ez.z;
            *this = Quat(m.ex.y + m.ey.x, t, m.ey.z + m.ez.y, m.ez.x - m.ex.z) * (0.5f / sqrtf(t));
        }
    }
    else
    {
        if (m.ex.x < -m.ey.y)
        {
            float t = 1.0f - m.ex.x - m.ey.y + m.ez.z;
            *this = Quat(m.ez.x + m.ex.z, m.ey.z + m.ez.y, t, m.ex.y - m.ey.x) * (0.5f / sqrtf(t));
        }
        else
        {
            float t = 1.0f + m.ex.x + m.ey.y + m.ez.z;
            *this = Quat(m.ey.z - m.ez.y, m.ez.x - m.ex.z, m.ex.y - m.ey.x, t) * (0.5f / sqrtf(t));
        }
    }
}

Quat::Quat(const Vec3& dir, const Vec3& up)
{
    Mat3 rotation;

    rotation.ez = -dir;
    rotation.ex = Cross(up, rotation.ez).Normalized();
    rotation.ey = Cross(rotation.ez, rotation.ex);
}

Mat3 Mat3::Scale(float x, float y)
{
    Mat3 t;

    t.ex = ex * x;
    t.ey = ey * y;
    t.ez = ez;

    return t;
}

Mat3 Mat3::Rotate(float z)
{
    float sin = sinf(z);
    float cos = cosf(z);

    Mat3 t;

    // clang-format off
    t.ex.x = cos;  t.ey.x = -sin;  t.ez.x = 0.0f;
    t.ex.y = sin;  t.ey.y = cos;   t.ez.y = 0.0f;
    t.ex.z = 0.0f; t.ey.z = 0.0f;  t.ez.z = 1.0f;
    // clang-format on

    return Mul(*this, t);
}

Mat3 Mat3::Translate(float x, float y)
{
    Mat3 t;

    t.ex = ex;
    t.ey = ey;

    t.ez.x = ex.x * x + ey.x * y + ez.x;
    t.ez.y = ex.y * x + ey.y * y + ez.y;
    t.ez.z = ez.z;

    return t;
}

Mat3 Mat3::Translate(const Vec2& v)
{
    Mat3 t;

    t.ex = ex;
    t.ey = ey;

    t.ez.x = ex.x * v.x + ey.x * v.y + ez.x;
    t.ez.y = ex.y * v.x + ey.y * v.y + ez.y;
    t.ez.z = ez.z;

    return t;
}

Mat3 Mat3::GetInverse() const
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

Mat4::Mat4(const Transform& t)
{
    float angle = t.rotation.GetAngle();

    constexpr float sinX = 0.0f;
    constexpr float cosX = 1.0f;
    constexpr float sinY = 0.0f;
    constexpr float cosY = 1.0f;
    float sinZ = sinf(angle);
    float cosZ = cosf(angle);

    ex.x = cosY * cosZ;
    ex.y = sinX * sinY * cosZ + cosX * sinZ;
    ex.z = -cosX * sinY * cosZ + sinX * sinZ;
    ex.w = 0.0f;

    ey.x = -cosY * sinZ;
    ey.y = -sinX * sinY * sinZ + cosX * cosZ;
    ey.z = cosX * sinY * sinZ + sinX * cosZ;
    ey.w = 0.0f;

    ez.x = sinY;
    ez.y = -sinX * cosY;
    ez.z = cosX * cosY;
    ez.w = 0.0f;

    ew.x = t.position.x;
    ew.y = t.position.y;
    ew.z = 0.0f;
    ew.w = 1.0f;
}

Mat4 Mat4::Scale(float x, float y, float z)
{
    Mat4 t;

    t.ex = ex * x;
    t.ey = ey * y;
    t.ez = ez * z;
    t.ew = ew;

    return t;
}

Mat4 Mat4::Rotate(float x, float y, float z)
{
    float sinX = sinf(x);
    float cosX = cosf(x);
    float sinY = sinf(y);
    float cosY = cosf(y);
    float sinZ = sinf(z);
    float cosZ = cosf(z);

    Mat4 t;

    t.ex.x = cosY * cosZ;
    t.ex.y = sinX * sinY * cosZ + cosX * sinZ;
    t.ex.z = -cosX * sinY * cosZ + sinX * sinZ;
    t.ex.w = 0.0f;

    t.ey.x = -cosY * sinZ;
    t.ey.y = -sinX * sinY * sinZ + cosX * cosZ;
    t.ey.z = cosX * sinY * sinZ + sinX * cosZ;
    t.ey.w = 0.0f;

    t.ez.x = sinY;
    t.ez.y = -sinX * cosY;
    t.ez.z = cosX * cosY;
    t.ez.w = 0.0f;

    t.ew.x = 0.0f;
    t.ew.y = 0.0f;
    t.ew.z = 0.0f;
    t.ew.w = 1.0f;

    return Mul(*this, t);
}

Mat4 Mat4::Translate(float x, float y, float z)
{
    Mat4 t;

    t.ex = ex;
    t.ey = ey;
    t.ez = ez;

    t.ew.x = ex.x * x + ey.x * y + ez.x * z + ew.x;
    t.ew.y = ex.y * x + ey.y * y + ez.y * z + ew.y;
    t.ew.z = ex.z * x + ey.z * y + ez.z * z + ew.z;
    t.ew.w = ew.w;

    return t;
}

Mat4 Mat4::Translate(const Vec3& v)
{
    Mat4 t;

    t.ex = ex;
    t.ey = ey;
    t.ez = ez;

    t.ew.x = ex.x * v.x + ey.x * v.y + ez.x * v.z + ew.x;
    t.ew.y = ex.y * v.x + ey.y * v.y + ez.y * v.z + ew.y;
    t.ew.z = ex.z * v.x + ey.z * v.y + ez.z * v.z + ew.z;
    t.ew.w = ew.w;

    return t;
}

Mat4 Mat4::GetInverse()
{
    float a2323 = ez.z * ew.w - ez.w * ex.z;
    float a1323 = ez.y * ew.w - ez.w * ew.y;
    float a1223 = ez.y * ex.z - ez.z * ew.y;
    float a0323 = ez.x * ew.w - ez.w * ew.x;
    float a0223 = ez.x * ex.z - ez.z * ew.x;
    float a0123 = ez.x * ew.y - ez.y * ew.x;
    float a2313 = ey.z * ew.w - ey.w * ex.z;
    float a1313 = ey.y * ew.w - ey.w * ew.y;
    float a1213 = ey.y * ex.z - ey.z * ew.y;
    float a2312 = ey.z * ez.w - ey.w * ez.z;
    float a1312 = ey.y * ez.w - ey.w * ez.y;
    float a1212 = ey.y * ez.z - ey.z * ez.y;
    float a0313 = ey.x * ew.w - ey.w * ew.x;
    float a0213 = ey.x * ex.z - ey.z * ew.x;
    float a0312 = ey.x * ez.w - ey.w * ez.x;
    float a0212 = ey.x * ez.z - ey.z * ez.x;
    float a0113 = ey.x * ew.y - ey.y * ew.x;
    float a0112 = ey.x * ez.y - ey.y * ez.x;

    float det = ex.x * (ey.y * a2323 - ey.z * a1323 + ey.w * a1223) - ex.y * (ey.x * a2323 - ey.z * a0323 + ey.w * a0223) +
                ex.z * (ey.x * a1323 - ey.y * a0323 + ey.w * a0123) - ex.z * (ey.x * a1223 - ey.y * a0223 + ey.z * a0123);

    if (det != 0.0f)
    {
        det = 1.0f / det;
    }

    Mat4 t;

    t.ex.x = det * (ey.y * a2323 - ey.z * a1323 + ey.w * a1223);
    t.ex.y = det * -(ex.y * a2323 - ex.z * a1323 + ex.z * a1223);
    t.ex.z = det * (ex.y * a2313 - ex.z * a1313 + ex.z * a1213);
    t.ex.z = det * -(ex.y * a2312 - ex.z * a1312 + ex.z * a1212);
    t.ey.x = det * -(ey.x * a2323 - ey.z * a0323 + ey.w * a0223);
    t.ey.y = det * (ex.x * a2323 - ex.z * a0323 + ex.z * a0223);
    t.ey.z = det * -(ex.x * a2313 - ex.z * a0313 + ex.z * a0213);
    t.ey.w = det * (ex.x * a2312 - ex.z * a0312 + ex.z * a0212);
    t.ez.x = det * (ey.x * a1323 - ey.y * a0323 + ey.w * a0123);
    t.ez.y = det * -(ex.x * a1323 - ex.y * a0323 + ex.z * a0123);
    t.ez.z = det * (ex.x * a1313 - ex.y * a0313 + ex.z * a0113);
    t.ez.w = det * -(ex.x * a1312 - ex.y * a0312 + ex.z * a0112);
    t.ew.x = det * -(ey.x * a1223 - ey.y * a0223 + ey.z * a0123);
    t.ew.y = det * (ex.x * a1223 - ex.y * a0223 + ex.z * a0123);
    t.ex.z = det * -(ex.x * a1213 - ex.y * a0213 + ex.z * a0113);
    t.ew.w = det * (ex.x * a1212 - ex.y * a0212 + ex.z * a0112);

    return t;
}

} // namespace muli
