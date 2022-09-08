#pragma once

#include <glm/glm.hpp>
#include <math.h>

namespace spe
{

struct Rotation
{
    float angle = 0.0f;

    // sine, cosine
    float s = 0.0f;
    float c = 1.0f;

    Rotation() = default;

    explicit Rotation(float _angle)
    {
        angle = _angle;
        s = sinf(_angle);
        c = cosf(_angle);
    }

    void operator+=(float _angle)
    {
        angle += _angle;
        s = sinf(angle);
        c = cosf(angle);
    }

    void operator-=(float _angle)
    {
        angle -= _angle;
        s = sinf(angle);
        c = cosf(angle);
    }

    void operator=(float _angle)
    {
        angle = _angle;
        s = sinf(angle);
        c = cosf(angle);
    }

    void SetIdentity()
    {
        angle = 0.0f;
        s = 0.0f;
        c = 1.0f;
    }

    float GetAngle() const
    {
        return angle;
    }
};

struct Transform
{
    glm::vec2 position{ 0.0f, 0.0f };
    Rotation rotation{};

    Transform() = default;

    Transform(const glm::vec2& _position, const Rotation& _rotation)
        : position{ _position }
        , rotation{ _rotation }
    {
    }

    void Set(const glm::vec2& _position, const Rotation& _rotation)
    {
        position = _position;
        rotation = _rotation;
    }

    void SetIdentity()
    {
        position.x = 0.0f;
        position.y = 0.0f;
        rotation.SetIdentity();
    }
};

// Rotate a vector
inline glm::vec2 mul(const Rotation& r, const glm::vec2& v)
{
    return glm::vec2{ r.c * v.x - r.s * v.y, r.s * v.x + r.c * v.y };
}

inline glm::vec2 operator*(const Rotation& r, const glm::vec2& v)
{
    return glm::vec2{ r.c * v.x - r.s * v.y, r.s * v.x + r.c * v.y };
}

// Inverse rotate a vector
inline glm::vec2 mul_t(const Rotation& r, const glm::vec2& v)
{
    return glm::vec2{ r.c * v.x + r.s * v.y, -r.s * v.x + r.c * v.y };
}

// Transform a vector: simplified matrix multiplication
inline glm::vec2 mul(const Transform& t, const glm::vec2& v)
{
    float x = (t.rotation.c * v.x - t.rotation.s * v.y) + t.position.x;
    float y = (t.rotation.s * v.x + t.rotation.c * v.y) + t.position.y;

    return glm::vec2{ x, y };
}

inline glm::vec2 operator*(const Transform& t, const glm::vec2& v)
{
    float x = (t.rotation.c * v.x - t.rotation.s * v.y) + t.position.x;
    float y = (t.rotation.s * v.x + t.rotation.c * v.y) + t.position.y;

    return glm::vec2{ x, y };
}

// Inverse transform a vector
inline glm::vec2 mul_t(const Transform& t, const glm::vec2& v)
{
    float px = v.x - t.position.x;
    float py = v.y - t.position.y;
    float x = (t.rotation.c * px + t.rotation.s * py);
    float y = (-t.rotation.s * px + t.rotation.c * py);

    return glm::vec2{ x, y };
}

} // namespace spe
