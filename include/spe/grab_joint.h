#pragma once

#include "common.h"
#include "joint.h"

namespace spe
{
struct Settings;

class GrabJoint : public Joint
{
public:
    GrabJoint(
        RigidBody* _body,
        glm::vec2 _anchor,
        glm::vec2 _target,
        const Settings& _settings,
        float _frequency = 1.0f,
        float _dampingRatio = 0.5f,
        float _jointMass = -1.0f
    );

    virtual void Prepare() override;
    virtual void Solve() override;

    inline const glm::vec2& GetLocalAnchor() const;
    inline const glm::vec2& GetTarget() const;
    inline void SetTarget(glm::vec2 target);

private:
    glm::vec2 localAnchor;
    glm::vec2 target;

    glm::vec2 r{ 0.0f };
    glm::mat2 m{ 1.0f };
    glm::vec2 bias{ 0.0f };
    glm::vec2 impulseSum{ 0.0f };

    void ApplyImpulse(const glm::vec2& lambda);
};

inline const glm::vec2& GrabJoint::GetLocalAnchor() const
{
    return localAnchor;
}

inline const glm::vec2& GrabJoint::GetTarget() const
{
    return target;
}

inline void GrabJoint::SetTarget(glm::vec2 _target)
{
    target = std::move(_target);
}

}