#pragma once

#include "common.h"

namespace muli
{

struct DebugOptions
{
    bool pause = false;
    bool step = false;
    bool drawOutlineOnly = false;
    bool showBVH = false;
    bool showAABB = false;
    bool showContactPoint = false;
    bool showContactNormal = false;
    bool resetCamera = true;
};

enum UserFlag : uint32
{
    REMOVE_OUTLINE = 1 << 1,
    RENDER_POLYGON_RADIUS = 1 << 2,
};

} // namespace muli
