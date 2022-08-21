#pragma once

#include "common.h"
#include "joint.h"

namespace spe
{
struct Settings;

class DistanceJoint : public Joint
{
public:
    DistanceJoint(
        RigidBody* _bodyA,
        RigidBody* _bodyB,
        glm::vec2 _anchorA,
        glm::vec2 _anchorB,
        float _length,
        const Settings& _settings,
        float _frequency = 10.0f,
        float _dampingRatio = 1.0f,
        float _jointMass = 1.0f
    );

    virtual void Prepare() override;
    virtual void Solve() override;

    inline const glm::vec2& GetLocalAnchorA() const;
    inline const glm::vec2& GetLocalAnchorB() const;
    inline float GetJointLength() const;
    inline void SetJointLength(float _length);

private:
    glm::vec2 localAnchorA;
    glm::vec2 localAnchorB;

    float length{ 0.0f };

    glm::vec2 ra{ 0.0f };
    glm::vec2 rb{ 0.0f };

    float m{ 1.0f };
    glm::vec2 n{ 0.0f };
    float bias{ 0.0f };
    float impulseSum{ 0.0f };

    void ApplyImpulse(float lambda);
};

inline const glm::vec2& DistanceJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const glm::vec2& DistanceJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

inline float DistanceJoint::GetJointLength() const
{
    return length;
}

inline void DistanceJoint::SetJointLength(float _length)
{
    length = glm::clamp<float>(length, 0, FLT_MAX);
}

}