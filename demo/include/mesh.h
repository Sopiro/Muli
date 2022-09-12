#pragma once

#include "common.h"

namespace spe
{

class Mesh
{
public:
    Mesh(const std::vector<Vec3>& _vertices, const std::vector<Vec2>& _texCoords, const std::vector<uint32_t>& _indices);
    ~Mesh() noexcept;

    Mesh(const Mesh&) noexcept = delete;
    Mesh& operator=(const Mesh&) noexcept = delete;

    Mesh(Mesh&&) noexcept;
    Mesh& operator=(Mesh&&) noexcept = delete;

    void Draw(GLenum drawMode = GL_TRIANGLES);

private:
    bool moved = false;

    std::vector<Vec3> vertices;
    std::vector<Vec2> texCoords;
    std::vector<uint32_t> indices;

    uint32_t VBOv;
    uint32_t VBOt;
    uint32_t VAO;
    uint32_t EBOt;
    uint32_t EBOl;
};

} // namespace spe