#include "spe/world.h"
#include "spe/island.h"
#include <iostream>

namespace spe
{

World::World(const Settings& simulationSettings) :
	settings{ simulationSettings }
{
	bodies.reserve(DEFAULT_BODY_RESERVE_COUNT);
}

World::~World() noexcept
{
	Reset();
}

void World::Step(float dt)
{
	settings.DT = dt;
	settings.INV_DT = 1.0f / dt;

	for (size_t i = 0; i < bodies.size(); i++)
	{
		RigidBody* b = bodies[i];
		b->manifoldIDs.clear();

		if (b->sleeping) continue;
		if (b->type == BodyType::Static) b->sleeping = true;

		Node* node = b->node;
		AABB tightAABB = create_AABB(b, 0.0f);

		if (contains_AABB(node->aabb, tightAABB)) continue;

		tree.Remove(b);
		tree.Insert(b);
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

		if (a->type == BodyType::Static && b->type == BodyType::Static)
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
	uint32_t restingBodies = 0;
	uint32_t islandID = 0;
	sleepingIslands = 0;
	sleepingBodies = 0;

	std::unordered_set<uint32_t> visited{};
	std::stack<RigidBody*> stack;

	// Perform a DFS(Depth First Search) on the constraint graph
	// After building island, each island can be solved in parallel because they are independent of each other
	for (size_t i = 0; i < bodies.size(); i++)
	{
		RigidBody* b = bodies[i];

		if (b->type == BodyType::Static || (visited.find(b->id) != visited.end()))
			continue;

		stack = std::stack<RigidBody*>();
		stack.push(b);

		islandID++;
		while (stack.size() > 0)
		{
			RigidBody* t = stack.top();
			stack.pop();

			if (t->type == BodyType::Static || (visited.find(t->id) != visited.end()))
				continue;

			visited.insert(t->id);
			t->islandID = islandID;
			island.bodies.push_back(t);

			for (size_t c = 0; c < t->manifoldIDs.size(); c++)
			{
				uint32_t key = t->manifoldIDs[c];
				ContactConstraint* cc = contactConstraintMap[key];

				RigidBody* other = cc->bodyB->id == t->id ? cc->bodyA : cc->bodyB;

				if (visited.find(other->id) != visited.end())
					continue;

				island.ccs.push_back(cc);
				stack.push(other);
			}

			for (size_t j = 0; j < t->jointIDs.size(); j++)
			{
				uint32_t key = t->jointIDs[j];
				Joint* joint = jointMap[key];

				RigidBody* other = joint->bodyB->id == t->id ? joint->bodyA : joint->bodyB;

				if (joint->bodyA == joint->bodyB)
				{
					island.js.push_back(joint);
					t->Awake();
				}

				if (visited.find(other->id) != visited.end())
					continue;

				island.js.push_back(joint);
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

		island.Solve();
		island.Clear();
		restingBodies = 0;
	}

	numIslands = islandID;
}

void World::Reset()
{
	uid = 0;

	for (RigidBody* body : bodies) delete body;
	for (Joint* joint : joints) delete joint;

	bodies.clear();
	pairs.clear();

	contactConstraints.clear();
	contactConstraintMap.clear();
	newContactConstraints.clear();
	newContactConstraintMap.clear();

	joints.clear();
	jointMap.clear();

	passTestSet.clear();

	tree.Reset();
}

void World::Add(RigidBody* body)
{
	if (body->world != nullptr)
		throw std::exception("This body is already registered.");

	body->world = this;
	body->id = ++uid;
	bodies.push_back(body);
	tree.Insert(body);
}

void World::Add(const std::vector<RigidBody*>& bodies)
{
	for (auto b : bodies)
	{
		Add(b);
	}
}

void World::Remove(RigidBody* body)
{
	auto it = std::find(bodies.begin(), bodies.end(), body);
	if (it == bodies.end()) throw std::exception("This body is not registered in this world.");;

	for (size_t i = 0; i < body->manifoldIDs.size(); i++)
	{
		uint32_t key = body->manifoldIDs[i];
		ContactConstraint* cc = contactConstraintMap[key];

		RigidBody* other = cc->bodyB->id == body->id ? cc->bodyA : cc->bodyB;
		other->Awake();
	}

	for (size_t i = 0; i < body->jointIDs.size(); i++)
	{
		uint32_t key = body->jointIDs[i];
		Joint* joint = jointMap[key];
		RigidBody* other = joint->bodyB->id == body->id ? joint->bodyA : joint->bodyB;

		other->Awake();

		auto it = std::find(other->jointIDs.begin(), other->jointIDs.end(), key);
		if (it != other->jointIDs.end())
		{
			std::iter_swap(it, other->jointIDs.end() - 1);
			other->jointIDs.pop_back();
		}

		jointMap.erase(key);
	}

	joints.clear();
	std::transform(jointMap.begin(), jointMap.end(), std::back_inserter(joints), [](auto& kv) { return kv.second;});

	bodies.erase(it);
	tree.Remove(body);

	delete body;
}

void World::Remove(const std::vector<RigidBody*>& bodies)
{
	for (size_t i = 0; i < bodies.size(); i++)
	{
		Remove(bodies[i]);
	}
}

void World::Remove(Joint* joint)
{
	if (std::find(joints.begin(), joints.end(), joint) == joints.end())
		throw std::exception("This joint is not registered in this world.");;

	auto it = std::find(joint->bodyA->jointIDs.begin(), joint->bodyA->jointIDs.end(), joint->id);
	if (it != joint->bodyA->jointIDs.end())
	{
		std::iter_swap(it, joint->bodyA->jointIDs.end() - 1);
		joint->bodyA->jointIDs.pop_back();
	}
	joint->bodyA->Awake();

	it = std::find(joint->bodyB->jointIDs.begin(), joint->bodyB->jointIDs.end(), joint->id);
	if (it != joint->bodyB->jointIDs.end())
	{
		std::iter_swap(it, joint->bodyB->jointIDs.end() - 1);
		joint->bodyB->jointIDs.pop_back();
	}
	joint->bodyB->Awake();

	jointMap.erase(joint->id);
	RemovePassTestPair(joint->bodyA, joint->bodyB);

	auto jit = std::find(joints.begin(), joints.end(), joint);
	if (jit != joints.end())
	{
		std::iter_swap(jit, joints.end() - 1);
		joints.pop_back();
	}

	delete joint;
}

void World::Remove(const std::vector<Joint*>& joints)
{
	for (size_t i = 0; i < joints.size(); i++)
	{
		Remove(joints[i]);
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

Box* World::CreateBox(float size, BodyType type, float density)
{
	return CreateBox(size, size, type, density);
}

Box* World::CreateBox(float width, float height, BodyType type, float density)
{
	Box* b = new Box(width, height, type, density);
	Add(b);
	return b;
}

Circle* World::CreateCircle(float radius, BodyType type, float density)
{
	Circle* c = new Circle(radius, type, density);
	Add(c);
	return c;
}

spe::Polygon* World::CreatePolygon(std::vector<glm::vec2> vertices, BodyType type, bool resetPosition, float density)
{
	Polygon* p = new Polygon(std::move(vertices), type, resetPosition, density);
	Add(p);
	return p;
}

Polygon* World::CreateRandomConvexPolygon(float radius, uint32_t num_vertices, float density)
{
	if (num_vertices < 3)
		num_vertices = glm::linearRand<uint32_t>(3, 8);

	std::vector<float> angles{};
	angles.reserve(num_vertices);

	for (size_t i = 0; i < num_vertices; i++)
	{
		angles.push_back(glm::linearRand<float>(0.0f, 1.0f) * glm::pi<float>() * 2.0f);
	}

	std::sort(angles.begin(), angles.end());

	std::vector<glm::vec2> vertices{};
	vertices.reserve(num_vertices);

	for (size_t i = 0; i < num_vertices; i++)
	{
		vertices.emplace_back(glm::cos(angles[i]) * radius, glm::sin(angles[i]) * radius);
	}

	Polygon* p = new Polygon(vertices, Dynamic, true, density);
	Add(p);
	return p;
}

Polygon* World::CreateRegularPolygon(float radius, uint32_t num_vertices, float initial_angle, float density)
{
	if (num_vertices < 3) num_vertices = glm::linearRand<uint32_t>(3, 11);

	float angleStart = initial_angle;
	float angle = glm::pi<float>() * 2.0f / num_vertices;

	if (num_vertices % 2 == 0)
		angleStart += angle / 2.0f;

	std::vector<glm::vec2> vertices;
	vertices.reserve(num_vertices);

	for (size_t i = 0; i < num_vertices; i++)
	{
		float currentAngle = angleStart + angle * i;

		glm::vec2 corner = glm::vec2{ glm::cos(currentAngle), glm::sin(currentAngle) };
		corner *= radius * glm::sqrt(2);

		vertices.push_back(corner);
	}

	Polygon* p = new Polygon(vertices, Dynamic, true, density);
	Add(p);
	return p;
}

GrabJoint* World::CreateGrabJoint(RigidBody* body, glm::vec2 anchor, glm::vec2 target, float frequency, float dampingRatio, float jointMass)
{
	if (body->world != this)
		throw std::exception("You should register the rigid bodies before registering the joint");

	GrabJoint* gj = new GrabJoint(body, anchor, target, settings, frequency, dampingRatio, jointMass);
	gj->id = ++uid;

	joints.push_back(gj);
	body->jointIDs.push_back(gj->id);

	jointMap.insert({ gj->id, gj });

	return gj;
}

RevoluteJoint* World::CreateRevoluteJoint(RigidBody* bodyA, RigidBody* bodyB, glm::vec2 anchor, float frequency, float dampingRatio, float jointMass)
{
	if (bodyA->world != this || bodyB->world != this)
		throw std::exception("You should register the rigid bodies before registering the joint");

	RevoluteJoint* rj = new RevoluteJoint(bodyA, bodyB, anchor, settings, frequency, dampingRatio, jointMass);
	rj->id = ++uid;

	joints.push_back(rj);
	bodyA->jointIDs.push_back(rj->id);
	bodyB->jointIDs.push_back(rj->id);

	jointMap.insert({ rj->id, rj });

	return rj;
}

DistanceJoint* World::CreateDistanceJoint(RigidBody* bodyA, RigidBody* bodyB, glm::vec2 anchorA, glm::vec2 anchorB, float length, float frequency, float dampingRatio, float jointMass)
{
	if (bodyA->world != this || bodyB->world != this)
		throw std::exception("You should register the rigid bodies before registering the joint");

	DistanceJoint* dj = new DistanceJoint(bodyA, bodyB, anchorA, anchorB, length, settings, frequency, dampingRatio, jointMass);
	dj->id = ++uid;

	joints.push_back(dj);
	bodyA->jointIDs.push_back(dj->id);
	bodyB->jointIDs.push_back(dj->id);

	jointMap.insert({ dj->id, dj });

	return dj;
}

DistanceJoint* World::CreateDistanceJoint(RigidBody* bodyA, RigidBody* bodyB, float length, float frequency, float dampingRatio, float jointMass)
{
	return CreateDistanceJoint(bodyA, bodyB, bodyA->position, bodyB->position, length, frequency, dampingRatio, jointMass);
}

}