#pragma once

#include "common.h"
#include "mesh.h"

namespace muli
{

std::unique_ptr<Mesh> GenerateMesh(const Collider* body, int32 circlePolygonCount = 13);

Vec3 rgb2hsl(float r, float g, float b);
Vec3 hsl2rgb(float h, float s, float l);

void PrintSizes();

} // namespace muli