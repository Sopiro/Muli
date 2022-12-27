#pragma once

#include "aabb.h"

namespace muli
{

constexpr int32 max_contact_points = 2;
constexpr int32 max_local_polygon_vertices = 8;

constexpr float linear_slop = 0.002f; // meters
constexpr float position_solver_threshold = linear_slop * 4.0f;
constexpr float toi_position_solver_threshold = linear_slop * 2.0f;
constexpr float position_correction = 0.2f; // The baumgarte term (0.0 ~ 1.0)
constexpr float toi_position_correction = 0.6f;
constexpr float max_position_correction = 0.2f; // meters

// Collision detection settings
constexpr int32 gjk_max_iteration = 20;
constexpr float gjk_tolerance = epsilon * epsilon;
constexpr int32 epa_max_iteration = 20;
constexpr float epa_tolerance = epsilon;
constexpr float contact_merge_threshold = linear_slop * 0.001f;

// Continuous simulation settings
constexpr int32 max_sub_steps = 8;
constexpr int32 max_toi_contacts = 32;

// Broad phase settings
constexpr Vec2 aabb_margin{ 0.05f };
constexpr float aabb_multiplier = 4.0f;

// Default body/collider settings

// Radius must be greater than 2.0 * linear_slop for stable CCD
// Otherwise, TOI solver will never converge to a safe position

// While minimum_radius is the minimum radius required for stable CCD,
// it's not recommended to use it as the minimum value.
// Instead, it is advisable to use the default_radius as the minimum radius.
constexpr float minimum_radius = linear_slop * 2.0f + linear_slop * 0.05f;
constexpr float default_radius = linear_slop * 3.0f;
constexpr float default_density = 12.5f;
constexpr float default_friction = 0.5f;
constexpr float default_restitution = 0.0f;
constexpr float default_restitution_treshold = 2.0f; // m/s
constexpr float default_surface_speed = 0.0f;

// Default joint settings
constexpr float default_joint_frequency = 10.0f;
constexpr float default_joint_damping_ratio = 1.0f;
constexpr float default_joint_mass = 1.0f;

// Simulation settings
struct WorldSettings
{
    bool apply_gravity = true;
    Vec2 gravity{ 0.0f, -10.0f };

    int32 velocity_iterations = 8;
    int32 position_iterations = 3;

    bool sleeping = true;
    float sleeping_treshold = 0.5f;                                              // second
    float rest_linear_tolerance = 0.01f * 0.01f;                                 // (m/s)^2
    float reset_angular_tolerance = (0.5f * pi / 180.0f) * (0.5f * pi / 180.0f); // (rad/s)^2

    bool continuous = true;
    bool sub_stepping = false;

    AABB world_bounds{ Vec2{ -max_value, -max_value }, Vec2{ max_value, max_value } };

    mutable bool warm_starting = true;
    mutable float dt;
    mutable float inv_dt;
};

} // namespace muli