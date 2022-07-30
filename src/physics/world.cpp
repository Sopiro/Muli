#include "spe/physics/world.h"
#include "spe/physics/island.h"

namespace spe
{

World::World(const Settings& simulationSettings) :
	settings{ simulationSettings }
{
	bodies.reserve(256);
}

World::~World() noexcept
{
	for (RigidBody* body : bodies)
	{
		delete body;
	}
}

void World::Update(float dt)
{
	for (size_t i = 0; i < bodies.size(); i++)
	{
		RigidBody* b = bodies[i];
		b->manifoldIDs.clear();

		if (b->sleeping) continue;

		Node* node = b->node;
		AABB tightAABB = create_AABB(b, 0.0f);

		if (contains_AABB(node->aabb, tightAABB)) continue;

		tree.Remove(b);
		tree.Add(b);
	}

	// Broad Phase
	// Retrieve a list of collider pairs that are potentially colliding
	// pairs = get_collision_pair_n2(bodies);
	pairs.clear();
	tree.GetCollisionPairs(pairs);

	newContactConstraints.clear();
	newContactConstraintMap.clear();
	newContactConstraints.reserve(pairs.size());
	newContactConstraintMap.reserve(pairs.size());

	for (size_t i = 0; i < pairs.size(); i++)
	{
		std::pair<RigidBody*, RigidBody*> pair = pairs[i];
		RigidBody* a = pair.first;
		RigidBody* b = pair.second;

		// Improve coherence
		if (a->id > b->id)
		{
			a = pair.second;
			b = pair.first;
		}

		if (a->type == Static && b->type == Static)
			continue;

		uint32_t key = make_pair_natural(a->id, b->id);
		if (passTestSet.find(key) != passTestSet.end()) continue;

		// Narrow Phase
		// Execute more accurate and expensive collision detection
		std::optional<ContactManifold> newManifold = detect_collision(a, b);
		if (!newManifold.has_value()) continue;

		ContactConstraint& cc = newContactConstraints.emplace_back(std::move(newManifold.value()), settings);
		newContactConstraintMap.insert({ key, &cc });

		a->manifoldIDs.push_back(key);
		b->manifoldIDs.push_back(key);

		if (settings.WARM_STARTING)
		{
			auto it = contactConstraintMap.find(key);
			if (it != contactConstraintMap.end())
			{
				ContactConstraint* oldCC = it->second;
				cc.TryWarmStart(*oldCC);
			}
		}
	}

	contactConstraints = std::move(newContactConstraints);
	contactConstraintMap = std::move(newContactConstraintMap);

	// Build the constraint island
	Island island{ *this };
	int32_t restingBodies = 0;
	int32_t islandID = 0;
	sleepingIslands = 0;
	sleepingBodies = 0;

	std::unordered_set<int32_t> visited{};
	std::stack<RigidBody*> stack;

	// Perform a DFS(Depth First Search) on the constraint graph
	// After building island, each island can be solved in parallel because they are independent of each other
	for (size_t i = 0; i < bodies.size(); i++)
	{
		RigidBody* b = bodies[i];

		if (b->type == Static || (visited.find(b->id) != visited.end()))
			continue;

		stack = std::stack<RigidBody*>();
		stack.push(b);

		islandID++;
		while (stack.size() > 0)
		{
			RigidBody* t = stack.top();
			stack.pop();

			if (t->type == Static || (visited.find(t->id) != visited.end()))
				continue;

			visited.insert(t->id);
			t->islandID = islandID;
			island.bodies.push_back(t);

			for (size_t c = 0; c < t->manifoldIDs.size(); c++)
			{
				int32_t key = t->manifoldIDs[c];
				ContactConstraint* cc = contactConstraintMap[key];

				RigidBody* other = cc->bodyB->id == t->id ? cc->bodyA : cc->bodyB;

				if (visited.find(other->id) != visited.end())
					continue;

				island.ccs.push_back(cc);
				stack.push(other);
			}

			if (t->resting > settings.SLEEPING_TRESHOLD)
				restingBodies++;
		}

		island.sleeping = settings.SLEEPING_ENABLED && (restingBodies == island.bodies.size());

		if (island.sleeping)
		{
			sleepingBodies += island.bodies.size();
			sleepingIslands++;
		}

		island.Solve(dt);
		island.Clear();
		restingBodies = 0;
	}

	numIslands = islandID;
}

void World::Reset()
{
	uid = 0;
	tree.Reset();
	bodies.clear();
	pairs.clear();

	contactConstraints.clear();
	contactConstraintMap.clear();
	newContactConstraints.clear();
	newContactConstraintMap.clear();
	passTestSet.clear();
}

void World::Register(RigidBody* body)
{
	body->id = uid++;
	bodies.push_back(body);
	tree.Add(body);
}

void World::Register(const std::vector<RigidBody*>& bodies)
{
	for (auto b : bodies)
	{
		Register(b);
	}
}

void World::Unregister(RigidBody* body)
{
	auto it = std::find(bodies.begin(), bodies.end(), body);

	if (it != bodies.end())
	{
		for (size_t i = 0; i < body->manifoldIDs.size(); i++)
		{
			int32_t key = body->manifoldIDs[i];
			ContactConstraint* cc = contactConstraintMap[key];

			RigidBody* other = cc->bodyB->id == body->id ? cc->bodyA : cc->bodyB;
			other->Awake();
		}

		bodies.erase(it);
		tree.Remove(body);

		delete body;
	}
}

void World::Unregister(const std::vector<RigidBody*>& bodies)
{
	for (size_t i = 0; i < bodies.size(); i++)
	{
		Unregister(bodies[i]);
	}
}

std::vector<RigidBody*> World::QueryPoint(const glm::vec2& point) const
{
	std::vector<RigidBody*> res;
	std::vector<Node*> nodes = tree.QueryPoint(point);

	for (size_t i = 0; i < nodes.size(); i++)
	{
		RigidBody* body = nodes[i]->body;

		if (test_point_inside(body, point))
		{
			res.push_back(body);
		}
	}

	return res;
}

std::vector<RigidBody*> World::QueryRegion(const AABB& region) const
{
	std::vector<RigidBody*> res;
	std::vector<Node*> nodes = tree.QueryRegion(region);

	for (size_t i = 0; i < nodes.size(); i++)
	{
		RigidBody* body = nodes[i]->body;

		float w = region.max.x - region.min.x;
		float h = region.max.y - region.min.y;

		Polygon t{ {region.min, {region.max.x, region.min.y}, region.max, {region.min.x, region.max.y}}, Dynamic, false };

		if (detect_collision(body, &t))
		{
			res.push_back(body);
		}
	}

	return res;
}

const std::vector<RigidBody*>& World::GetBodies() const
{
	return bodies;
}

const size_t World::GetSleepingBodyCount() const
{
	return sleepingBodies;
}

const size_t World::GetSleepingIslandCount() const
{
	return sleepingIslands;
}

const AABBTree& World::GetBVH() const
{
	return tree;
}

const std::vector<ContactConstraint>& World::GetContactConstraints() const
{
	return contactConstraints;
}

Box* World::CreateBox(float width, float height, BodyType type, float density)
{
	Box* b = new Box(width, height, type, density);
	Register(b);
	return b;
}

Circle* World::CreateCircle(float radius, BodyType type, float density)
{
	Circle* c = new Circle(radius, type, density);
	Register(c);
	return c;
}

spe::Polygon* World::CreatePolygon(std::vector<glm::vec2> vertices, BodyType type, bool resetPosition, float density)
{
	Polygon* p = new Polygon(std::move(vertices), type, resetPosition, density);
	Register(p);
	return p;
}

}