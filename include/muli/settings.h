#pragma once

#include "aabb.h"

#define MAX_CONTACT_POINT 2

// Default broad phase settings
#define DEFAULT_AABB_MARGIN 0.05f
#define DEFAULT_AABB_MULTIPLIER 4.0f

// Default body settings
#define DEFAULT_RADIUS 0.005f // stable if greater than 0.5 * PENETRATION_SLOP
#define DEFAULT_DENSITY 12.5f
#define DEFAULT_FRICTION 0.5f
#define DEFAULT_RESTITUTION 0.0f
#define DEFAULT_SURFACESPEED 0.0f

// Default joint settings
#define DEFAULT_FREQUENCY 10.0f
#define DEFAULT_DAMPING_RATIO 1.0f
#define DEFAULT_JOINT_MASS 1.0f

// Collision detection settings
#define GJK_MAX_ITERATION 20
#define GJK_TOLERANCE 1.43e-14f // ≈ MULI_EPSILON * MULI_EPSILON
#define EPA_MAX_ITERATION 20
#define EPA_TOLERANCE 1.2e-7f // ≈ MULI_EPSILON
#define TANGENT_MIN_LENGTH 0.01f
#define CONTACT_MERGE_THRESHOLD 1.415f * TANGENT_MIN_LENGTH

#define LINEAR_SLOP 0.005f

namespace muli
{

// Simulation settings
struct WorldSettings
{
    bool apply_gravity = true;
    Vec2 gravity{ 0.0f, -10.0f };

    bool warm_starting = true;
    float position_correction = 0.2f; // 0.0 ~ 1.0

    float penetration_slop = LINEAR_SLOP; // meter
    float restitution_slop = 0.2f;        // m/s

    bool block_solve = true;
    int32 velocity_iterations = 8;
    int32 position_iterations = 3;

    bool sleeping = true;
    float sleeping_treshold = 0.5f;                                                        // second
    float rest_linear_tolerance = 0.01f * 0.01f;                                           // (m/s)^2
    float reset_angular_tolerance = (0.5f * MULI_PI / 180.0f) * (0.5f * MULI_PI / 180.0f); // (rad/s)^2

    bool continuous = true;

    AABB world_bounds{ Vec2{ -FLT_MAX, -FLT_MAX }, Vec2{ FLT_MAX, FLT_MAX } };

    mutable float dt;
    mutable float inv_dt;
};

} // namespace muli