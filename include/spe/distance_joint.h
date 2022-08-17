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

    const glm::vec2& GetLocalAnchorA() const;
    const glm::vec2& GetLocalAnchorB() const;
    float GetJointLength() const;
    void SetJointLength(float _length);

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

}