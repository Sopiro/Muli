#pragma once

#include "aabb.h"

namespace muli
{

constexpr int32 max_contact_points = 2;
constexpr int32 max_local_polygon_vertices = 8;

constexpr float linear_slop = 0.002f;
constexpr float position_solver_threshold = linear_slop * 4.0f;
constexpr float toi_position_solver_threshold = linear_slop * 2.0f;
constexpr float position_correction = 0.2f; // The baumgarte term (0.0 ~ 1.0)
constexpr float toi_position_correction = 0.35f;

// Collision detection settings
constexpr int32 gjk_max_iteration = 20;
constexpr float gjk_tolerance = 1.43e-14f; // ≈ MULI_EPSILON * MULI_EPSILON;
constexpr int32 epa_max_iteration = 20;
constexpr float epa_tolerance = 1.2e-7f; // ≈ MULI_EPSILON;
constexpr float contact_merge_threshold = linear_slop * 0.001f;

// Continuous simulation settings
constexpr int32 max_sub_steps = 8;
constexpr int32 max_toi_contacts = 32;

// Broad phase settings
constexpr float aabb_margin = 0.05f;
constexpr float aabb_multiplier = 4.0f;

// Default body settings
constexpr float default_radius = linear_slop * 3.0f; // Must be greater than 2.0 * linear_slop
constexpr float default_density = 12.5f;
constexpr float default_friction = 0.5f;
constexpr float default_restitution = 0.0f;
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

    float restitution_slop = 0.2f; // m/s

    bool block_solve = true;
    int32 velocity_iterations = 8;
    int32 position_iterations = 3;

    bool sleeping = true;
    float sleeping_treshold = 0.5f;                                              // second
    float rest_linear_tolerance = 0.01f * 0.01f;                                 // (m/s)^2
    float reset_angular_tolerance = (0.5f * pi / 180.0f) * (0.5f * pi / 180.0f); // (rad/s)^2

    bool continuous = true;
    bool sub_stepping = false;

    AABB world_bounds{ Vec2{ -FLT_MAX, -FLT_MAX }, Vec2{ FLT_MAX, FLT_MAX } };

    mutable bool warm_starting = true;
    mutable float dt;
    mutable float inv_dt;
};

} // namespace muli