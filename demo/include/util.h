#pragma once

#include "common.h"
#include "mesh.h"

namespace muli
{

Vec3 RGBtoHSL(Vec3 rgb);
Vec3 HSLtoRGB(Vec3 hsl);

void PrintSizes();

} // namespace muli