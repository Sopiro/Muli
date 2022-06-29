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
        friend class AABBTree;

    public:
        RigidBody(BodyType _type);
        ~RigidBody() noexcept;

        RigidBody(RigidBody&) = delete;
        RigidBody& operator=(RigidBody&) = delete;

        RigidBody(RigidBody&& _other) noexcept;
        RigidBody& operator=(RigidBody&& _other) noexcept;

        const Node* GetNode();

        virtual void SetDensity(float d) = 0;
        virtual void SetMass(float m) = 0;
        virtual float GetArea() = 0;

        void Awake();

        float GetDensity();
        float GetMass();
        float GetInverseMass();
        float GetInertia();
        float GetInverseInertia();

        float GetFriction();
        void SetFriction(float _friction);

        float GetRestitution();
        void SetRestitution(float _restitution);

        float GetSurfaceSpeed();
        void SetSurfaceSpeed(float _surfaceSpeed);

        glm::vec2 GetLinearVelocity();
        void SetLinearVelocity(glm::vec2 _linearVelocity);

        float GetAngularVelocity();
        void SetAngularVelocity(float _angularVelocity);

        glm::vec2 GetForce();
        void SetForce(glm::vec2 _force);

        float GetTorque();
        void SetTorque(float _torque);

        BodyType GetType();

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
        Mesh* mesh{ nullptr };
    };
}