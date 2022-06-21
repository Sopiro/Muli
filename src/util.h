#ifndef __UTIL_H__
#define __UTIL_H__

#include "common.h"

float CalculateConvexPolygonInertia(const std::vector<glm::vec2>& vertices, float mass, float area);

inline float CalculateCircleInertia(float radius, float mass)
{
    return mass * radius * radius / 2.0f;
}
#endif