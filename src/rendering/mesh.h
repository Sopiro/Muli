#pragma once

#include "../common.h"

namespace spe
{
    class Mesh
    {
    public:
        Mesh(std::vector<glm::vec3> _vertices, std::vector<glm::vec2> _texCoords, std::vector<uint32_t> _indices);
        ~Mesh() noexcept;
        
        Mesh(const Mesh&) noexcept = delete;
        Mesh& operator=(const Mesh&) noexcept = delete;
        
        Mesh(Mesh&&) noexcept;
        Mesh& operator=(Mesh&&) noexcept = delete;

        void Draw();

    private:
        bool moved = false; 

        std::vector<glm::vec3> vertices;
        std::vector<glm::vec2> texCoords;
        std::vector<uint32_t> indices;

        uint32_t VBOv;
        uint32_t VBOt;
        uint32_t VAO;
        uint32_t EBO;
    };
}