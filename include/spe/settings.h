#pragma once

namespace spe
{

// Default body settings
#define DEFAULT_DENSITY 12.5f
#define DEFAULT_FRICTION 0.7f
#define DEFAULT_RESTITUTION 0.0f
#define DEFAULT_SURFACESPEED 0.0f

// Collision detection settings
#define GJK_MAX_ITERATION 20
#define GJK_TOLERANCE 1e-17f
#define EPA_MAX_ITERATION 20
#define EPA_TOLERANCE 1e-13f
#define TANGENT_MIN_LENGTH 0.01f
#define CONTACT_MERGE_THRESHOLD 1.415f * TANGENT_MIN_LENGTH

}