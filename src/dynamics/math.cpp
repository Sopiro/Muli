#include "spe/math.h"

namespace spe
{

Mat4 Mat4::Scale(float x, float y, float z)
{
    Mat4 t{ 1.0f };

    t.ex.x = x;
    t.ey.y = y;
    t.ez.z = z;

    return mul(*this, t);
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
    t.ex.w = 0;

    t.ey.x = -cosY * sinZ;
    t.ey.y = -sinX * sinY * sinZ + cosX * cosZ;
    t.ey.z = cosX * sinY * sinZ + sinX * cosZ;
    t.ey.w = 0;

    t.ez.x = sinY;
    t.ez.y = -sinX * cosY;
    t.ez.z = cosX * cosY;
    t.ez.w = 0;

    t.ew.x = 0;
    t.ew.y = 0;
    t.ew.z = 0;
    t.ew.w = 1;

    return mul(*this, t);
}

Mat4 Mat4::Translate(float x, float y, float z)
{
    Mat4 t{ 1.0f };

    t.ew.x = x;
    t.ew.y = y;
    t.ew.z = z;

    return mul(*this, t);
}

Mat4 Mat4::Translate(const Vec3& v)
{
    Mat4 t{ 1.0f };

    t.ew.x = v.x;
    t.ew.y = v.y;
    t.ew.z = v.z;

    return mul(*this, t);
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

} // namespace spe
