#ifndef __MESH_H__
#define __MESH_H__

#include "../common.h"

namespace spe
{
    class Mesh
    {
    public:
        Mesh(std::vector<glm::vec3> _vertices, std::vector<glm::vec2> _texCoords, std::vector<uint32_t> _indices);
        ~Mesh();
        Mesh(Mesh& other) = delete;
        Mesh(const Mesh&) = delete;
        Mesh(Mesh&&) = delete;
        Mesh& operator=(const Mesh&) = delete;
        Mesh& operator=(Mesh&&) = delete;

        void Draw();

    private:
        std::vector<glm::vec3> vertices;
        std::vector<glm::vec2> texCoords;
        std::vector<uint32_t> indices;

        uint32_t VBOv;
        uint32_t VBOt;
        uint32_t VAO;
        uint32_t EBO;
    };
}

#endif