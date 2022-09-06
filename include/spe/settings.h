#pragma once

namespace spe
{

#define MAX_CONTACT_POINT 2

// Default broad phase settings
#define DEFAULT_AABB_MARGIN 0.05f
#define DEFAULT_VELOCITY_MULTIPLIER 4.0f

// Default body settings
#define DEFAULT_DENSITY 12.5f
#define DEFAULT_FRICTION 0.5f
#define DEFAULT_RESTITUTION 0.0f
#define DEFAULT_SURFACESPEED 0.0f

// Default joint settings
#define DEFAULT_FREQUENCY 15.0f
#define DEFAULT_DAMPING_RATIO 1.0f
#define DEFAULT_JOINT_MASS 1.0f

// Collision detection settings
#define GJK_MAX_ITERATION 20
#define GJK_TOLERANCE 1e-17f
#define EPA_MAX_ITERATION 20
#define EPA_TOLERANCE 1e-13f
#define TANGENT_MIN_LENGTH 0.01f
#define CONTACT_MERGE_THRESHOLD 1.415f * TANGENT_MIN_LENGTH

} // namespace spe