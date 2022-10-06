#pragma once

#include "aabb.h"

#define MAX_CONTACT_POINT 2

// Default broad phase settings
#define DEFAULT_AABB_MARGIN 0.05f
#define DEFAULT_VELOCITY_MULTIPLIER 4.0f

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
// FLT_EPSILON == 1.192092896e-07f
#define GJK_MAX_ITERATION 20
#define GJK_TOLERANCE 1.43e-14f // ≈ FLT_EPSILON * FLT_EPSILON
#define EPA_MAX_ITERATION 20
#define EPA_TOLERANCE 1.2e-7f // ≈ FLT_EPSILON
#define TANGENT_MIN_LENGTH 0.01f
#define CONTACT_MERGE_THRESHOLD 1.415f * TANGENT_MIN_LENGTH

namespace muli
{

// Simulation settings
struct WorldSettings
{
    mutable float DT = 1.0f / 60.0f;
    mutable float INV_DT = 60.0f;

    bool APPLY_GRAVITY = true;
    Vec2 GRAVITY{ 0.0f, -10.0f };

    bool WARM_STARTING = true;
    float POSITION_CORRECTION_BETA = 0.2f; // 0.0 ~ 1.0

    float PENETRATION_SLOP = 0.005f; // meter
    float RESTITUTION_SLOP = 0.2f;   // m/s

    bool BLOCK_SOLVE = true;
    uint32 VELOCITY_SOLVE_ITERATIONS = 8;
    uint32 POSITION_SOLVE_ITERATIONS = 3;

    bool SLEEPING = true;
    float SLEEPING_TRESHOLD = 0.5f;                                                       // second
    float REST_LINEAR_TOLERANCE = 0.01f * 0.01f;                                          // (m/s)^2
    float REST_ANGULAR_TOLERANCE = (0.5f * MULI_PI / 180.0f) * (0.5f * MULI_PI / 180.0f); // (rad/s)^2

    AABB VALID_REGION{ Vec2{ -FLT_MAX, -FLT_MAX }, Vec2{ FLT_MAX, FLT_MAX } };
};

} // namespace muli