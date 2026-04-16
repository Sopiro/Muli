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

namespace UserFlag
{

enum Flag : size_t
{
    remove_outline = 1 << 1,
    render_polygon_radius = 1 << 2,
    hide_joint_anchors = 1 << 3,
};

inline void SetFlag(RigidBody* body, Flag flag, bool enabled)
{
    if (enabled)
    {
        body->UserData = (void*)((size_t)body->UserData | flag);
    }
    else
    {
        body->UserData = (void*)((size_t)body->UserData & ~flag);
    }
}

inline bool IsEnabled(const RigidBody* body, Flag flag)
{
    return ((size_t)body->UserData & flag) == flag;
}

inline void SetFlag(Joint* joint, Flag flag, bool enabled)
{
    if (enabled)
    {
        joint->UserData = (void*)((size_t)joint->UserData | flag);
    }
    else
    {
        joint->UserData = (void*)((size_t)joint->UserData & ~flag);
    }
}

inline bool IsEnabled(const Joint* joint, Flag flag)
{
    return ((size_t)joint->UserData & flag) == flag;
}

} // namespace UserFlag

} // namespace muli
