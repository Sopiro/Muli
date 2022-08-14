#include "mesh.h"
#include "common.h"

namespace spe
{

std::unique_ptr<Mesh> generate_mesh_from_rigidbody(RigidBody& body, uint32_t circle_polygon_count = 13);

std::vector<uint32_t> triangulate(const std::vector<glm::vec2>& vertices);

}