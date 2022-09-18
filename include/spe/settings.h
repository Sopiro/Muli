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
#define DEFAULT_FREQUENCY 15.0f
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

namespace spe
{

// Simulation settings
struct WorldSettings
{
    mutable float DT = 1.0f / 60.0f;
    mutable float INV_DT = 60.0f;

    bool APPLY_GRAVITY = true;
    Vec2 GRAVITY{ 0.0f, -10.0f };

    bool WARM_STARTING = true;
    bool APPLY_WARM_STARTING_THRESHOLD = false;
    float WARM_STARTING_THRESHOLD = 0.005f * 0.005f - FLT_EPSILON;

    bool POSITION_CORRECTION = true;
    float POSITION_CORRECTION_BETA = 0.2f;

    float PENETRATION_SLOP = 0.005f;
    float RESTITUTION_SLOP = 0.1f;

    bool BLOCK_SOLVE = true;
    uint32 VELOCITY_SOLVE_ITERATIONS = 8;
    uint32 POSITION_SOLVE_ITERATIONS = 3;

    float REST_LINEAR_TOLERANCE = 0.01f * 0.01f;
    float REST_ANGULAR_TOLERANCE = (0.5f * SPE_PI / 180.0f) * (0.5f * SPE_PI / 180.0f);

    bool SLEEPING_ENABLED = true;
    float SLEEPING_TRESHOLD = 0.5f;

    AABB VALID_REGION{ Vec2{ -FLT_MAX, -FLT_MAX }, Vec2{ FLT_MAX, FLT_MAX } };
};

} // namespace spe