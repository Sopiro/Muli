#pragma once

#include "../common.h"
#include "aabbtree.h"
#include "detection.h"
#include "rigidbody.h"

namespace spe
{
    class World final
    {
    public:
        World();
        ~World() noexcept;

        World(const World&) noexcept = delete;
        World& operator=(const World&) noexcept = delete;

        World(World&&) noexcept = delete;
        World& operator=(World&&) noexcept = delete;

        void Update(float inv_dt);
        void Reset();

        void Register(RigidBody* body);
        void Register(const std::vector<RigidBody*>& bodies);
        void Unregister(RigidBody* body);
        void Unregister(const std::vector<RigidBody*>& bodies);

        std::vector<RigidBody*> QueryPoint(const glm::vec2& point) const;
        std::vector<RigidBody*> QueryRegion(const AABB& region) const;

        const AABBTree& GetBVH() const;
    private:
        uint32_t uid{ 0 };

        // Dynamic AABB Tree for broad phase collision detection
        AABBTree tree{};
        // All registered rigid bodies
        std::vector<RigidBody*> bodies{};

        // Constraints to be solved
        std::vector<ContactManifold> manifolds{};

        std::unordered_map<int32_t, ContactManifold*> manioldMap{};
        std::unordered_set<int32_t> passTestSet{};
    };
}