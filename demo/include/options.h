#pragma once

#include "common.h"

namespace muli
{

struct DebugOptions
{
    bool pause = false;
    bool step = false;
    bool draw_body = true;
    bool draw_outlined = false;
    bool show_bvh = false;
    bool show_aabb = false;
    bool show_contact_point = false;
    bool show_contact_normal = false;
    bool reset_camera = true;
    bool colorize_island = true;
};

enum UserFlag : uint32
{
    remove_outline = 1 << 1,
    render_polygon_radius = 1 << 2,
};

} // namespace muli
