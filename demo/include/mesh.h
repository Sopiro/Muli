#pragma once

#include "common.h"

namespace muli
{

class Mesh
{
public:
    Mesh(const std::vector<Vec3>& vertices, const std::vector<Vec2>& texCoords, const std::vector<uint32>& indices);
    ~Mesh() noexcept;

    Mesh(const Mesh&) noexcept = delete;
    Mesh& operator=(const Mesh&) noexcept = delete;

    Mesh(Mesh&&) noexcept;
    Mesh& operator=(Mesh&&) noexcept;

    void Draw(GLenum drawMode = GL_TRIANGLES);

private:
    bool moved = false;

    std::vector<Vec3> vertices;
    std::vector<Vec2> texCoords;
    std::vector<uint32> indices;

    uint32 VBOv;
    uint32 VBOt;
    uint32 VAO;
    uint32 EBOt;
    uint32 EBOl;
};

} // namespace muli