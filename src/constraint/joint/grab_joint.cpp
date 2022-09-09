#include "spe/grab_joint.h"
#include "spe/world.h"

namespace spe
{

GrabJoint::GrabJoint(RigidBody* _body,
                     Vec2 _anchor,
                     Vec2 _target,
                     const WorldSettings& _settings,
                     float _frequency,
                     float _dampingRatio,
                     float _jointMass)
    : Joint(type = Joint::Type::JointGrab, _body, _body, _settings, _frequency, _dampingRatio, _jointMass)
{
    localAnchor = mul_t(_body->GetTransform(), _anchor);
    target = _target;
}

void GrabJoint::Prepare()
{
    // Calculate Jacobian J and effective mass M
    // J = [I, skew(r)]
    // M = (J · M^-1 · J^t)^-1

    r = bodyA->GetRotation() * localAnchor;
    Vec2 p = bodyA->GetPosition() + r;

    Mat2 k;

    k[0][0] = bodyA->invMass + bodyA->invInertia * r.y * r.y;
    k[1][0] = -bodyA->invInertia * r.y * r.x;
    k[0][1] = -bodyA->invInertia * r.x * r.y;
    k[1][1] = bodyA->invMass + bodyA->invInertia * r.x * r.x;

    k[0][0] += gamma;
    k[1][1] += gamma;

    m = k.GetInverse();

    Vec2 error = p - target;

    if (settings.POSITION_CORRECTION)
        bias = error * beta * settings.INV_DT;
    else
        bias.SetZero();

    if (settings.WARM_STARTING)
    {
        ApplyImpulse(impulseSum);
    }
}

void GrabJoint::SolveVelocityConstraint()
{
    // Calculate corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    Vec2 jv = bodyA->linearVelocity + cross(bodyA->angularVelocity, r);
    Vec2 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);

    impulseSum += lambda;
}

void GrabJoint::ApplyImpulse(const Vec2& lambda)
{
    bodyA->linearVelocity += lambda * bodyA->invMass;
    bodyA->angularVelocity += bodyA->invInertia * cross(r, lambda);
}

} // namespace spe