#pragma once

#include "math.h"

namespace muli
{

// Cubic Splines

template <typename T>
inline T SplineHermite(const T& p0, const T& p2, const T& t0, const T& t1, float t)
{
    float t2 = t * t;
    float t3 = t2 * t;

    float h00 = 2 * t3 - 3 * t2 + 1;
    float h10 = t3 - 2 * t2 + t;
    float h01 = -2 * t3 + 3 * t2;
    float h11 = t3 - t2;

    return h00 * p0 + h10 * t0 + h01 * p2 + h11 * t1;
}

template <typename T>
inline T SplineCatmullRom(const T& p0, const T& p2, const T& p3, const T& p4, float t)
{
    float t2 = t * t;
    float t3 = t2 * t;

    return 0.5f * ((2 * p2) + (-p0 + p3) * t + (2 * p0 - 5 * p2 + 4 * p3 - p4) * t2 + (-p0 + 3 * p2 - 3 * p3 + p4) * t3);
}

template <typename T>
inline T SplineCardinal(const T& p0, const T& p1, const T& p2, const T& p3, float t, float tension)
{
    // tension of 0 gives the same result as Catmull-Rom
    T t0 = (1 - tension) * (p2 - p0) * 0.5f;
    T t1 = (1 - tension) * (p3 - p1) * 0.5f;

    return SplineHermite(p1, p2, t0, t1, t);
}

template <typename T>
inline T SplineUniformBasis(const T& p0, const T& p1, const T& p2, const T& p3, float t)
{
    float t2 = t * t;
    float t3 = t * t2;

    float b0 = ((1 - t) * (1 - t) * (1 - t));
    float b1 = (3 * t3 - 6 * t2 + 4);
    float b2 = (-3 * t3 + 3 * t2 + 3 * t + 1.0f);
    float b3 = t3;

    return (b0 * p0 + b1 * p1 + b2 * p2 + b3 * p3) / 6.0f;
}

} // namespace muli
