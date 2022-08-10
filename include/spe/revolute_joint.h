#pragma once

#include "common.h"
#include "joint.h"

namespace spe
{
struct Settings;

class RevoluteJoint : public Joint
{
public:
    RevoluteJoint(
        RigidBody* _bodyA,
        RigidBody* _bodyB,
        glm::vec2 _anchor,
        const Settings& _settings,
        float _frequency = 10.0f,
        float _dampingRatio = 1.0f,
        float _jointMass = -1.0f
    );

    virtual void Prepare() override;
    virtual void Solve() override;

    const glm::vec2& GetLocalAnchorA() const;
    const glm::vec2& GetLocalAnchorB() const;

private:
    glm::vec2 localAnchorA;
    glm::vec2 localAnchorB;

    glm::vec2 ra{ 0.0f };
    glm::vec2 rb{ 0.0f };

    glm::mat2 m{ 1.0f };
    glm::vec2 bias{ 0.0f };
    glm::vec2 impulseSum{ 0.0f };

    void ApplyImpulse(const glm::vec2& lambda);
};
}