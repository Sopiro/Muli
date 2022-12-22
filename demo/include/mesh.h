#pragma once

#include "common.h"

namespace muli
{

class Mesh
{
public:
    Mesh(const std::vector<Vec3>& vertices, const std::vector<Vec2>& texCoords, const std::vector<int32>& indices);
    ~Mesh() noexcept;

    Mesh(const Mesh&) noexcept = delete;
    Mesh& operator=(const Mesh&) noexcept = delete;

    Mesh(Mesh&&) noexcept;
    Mesh& operator=(Mesh&&) noexcept;

    void Draw(GLenum drawMode = GL_TRIANGLES) const;

private:
    bool moved = false;

    std::vector<Vec3> vertices;
    std::vector<Vec2> texCoords;
    std::vector<int32> indices;

    GLuint VBOv;
    GLuint VBOt;
    GLuint VAO;
    GLuint EBOt;
    GLuint EBOl;
};

} // namespace muli