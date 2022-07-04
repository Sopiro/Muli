#pragma once

#include "../common.h"
#include "../entity.h"
#include "settings.h"
#include "../rendering/mesh.h"

namespace spe
{
    struct Node;

    enum BodyType : uint8_t
    {
        Static,
        Dynamic,
    };

    // Children: Polygon, Circle
    class RigidBody : public Entity
    {
        friend class World;
        friend class AABBTree;
        friend class ContactSolver;

    public:
        RigidBody(BodyType _type);
        ~RigidBody() noexcept;

        RigidBody(const RigidBody&) = delete;
        RigidBody& operator=(const RigidBody&) = delete;

        RigidBody(RigidBody&& _other) noexcept;
        RigidBody& operator=(RigidBody&& _other) noexcept;

        const Node* GetNode() const;

        virtual void SetDensity(float d) = 0;
        virtual void SetMass(float m) = 0;
        virtual float GetArea() const = 0;

        void Awake();

        float GetDensity() const;
        float GetMass() const;
        float GetInverseMass() const;
        float GetInertia() const;
        float GetInverseInertia() const;

        float GetFriction() const;
        void SetFriction(float _friction);

        float GetRestitution() const;
        void SetRestitution(float _restitution);

        float GetSurfaceSpeed() const;
        void SetSurfaceSpeed(float _surfaceSpeed);

        glm::vec2 GetLinearVelocity() const;
        void SetLinearVelocity(glm::vec2 _linearVelocity);

        float GetAngularVelocity() const;
        void SetAngularVelocity(float _angularVelocity);

        glm::vec2 GetForce() const;
        void SetForce(glm::vec2 _force);

        float GetTorque() const;
        void SetTorque(float _torque);

        BodyType GetType() const;

    protected:
        // Center of mass in local space = (0, 0)
        glm::vec2 force{ 0.0f };                        // N
        float torque{ 0.0f };                           // N⋅m

        glm::vec2 linearVelocity{ 0.0f };               // m/s
        float angularVelocity{ 0.0f };                  // rad/s

        float density;                                  // kg/m²
        float mass;                                     // kg
        float invMass;
        float inertia;                                  // kg⋅m²
        float invInertia;

        float friction{ DEFAULT_FRICTION };
        float restitution{ DEFAULT_RESTITUTION };
        float surfaceSpeed{ DEFAULT_SURFACESPEED };     // m/s (Tangential speed)

        BodyType type;

    private:
        bool moved{ false };

        int32_t id{ -1 };
        uint32_t islandID{ 0 };

        std::vector<uint32_t> manifoldIDs{};            // ids of contact manifold containing this body
        std::vector<uint32_t> jointIDs{};               // ids of the joint containing this body

        uint32_t resting{ 0 };
        bool sleeping{ false };

        Node* node{ nullptr };
    };
}