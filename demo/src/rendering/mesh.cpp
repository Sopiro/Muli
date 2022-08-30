#include "mesh.h"

namespace spe
{

Mesh::Mesh(std::vector<glm::vec3> _vertices, std::vector<glm::vec2> _texCoords, std::vector<uint32_t> _indices)
    : vertices{ std::move(_vertices) }
    , texCoords{ std::move(_texCoords) }
    , indices{ std::move(_indices) }
{
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBOv);
    glGenBuffers(1, &VBOt);
    glGenBuffers(1, &EBOt);
    glGenBuffers(1, &EBOl);

    // For better performance, merge vertices and texCoords vectors and define a shader vertex layout with only one VBO.
    glBindVertexArray(VAO);
    {
        // vertices
        glBindBuffer(GL_ARRAY_BUFFER, VBOv);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size() * 3, vertices.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, 0);
        glEnableVertexAttribArray(0);

        // tex coords
        glBindBuffer(GL_ARRAY_BUFFER, VBOt);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * texCoords.size() * 2, texCoords.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glEnableVertexAttribArray(1);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    glBindVertexArray(0);

    // indices for triangle
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOt);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * indices.size(), indices.data(), GL_STATIC_DRAW);

    std::vector<uint32_t> indices_l(vertices.size());
    std::iota(indices_l.begin(), indices_l.end(), 0);

    // indices for outline
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOl);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * indices_l.size(), indices_l.data(), GL_STATIC_DRAW);
}

Mesh::~Mesh()
{
    if (moved) return;

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBOv);
    glDeleteBuffers(1, &VBOt);
    glDeleteBuffers(1, &EBOt);
    glDeleteBuffers(1, &EBOl);
}

Mesh::Mesh(Mesh&& _m) noexcept
{
    _m.moved = true;

    vertices = std::move(_m.vertices);
    texCoords = std::move(_m.texCoords);
    indices = std::move(_m.indices);

    VAO = _m.VAO;
    VBOv = _m.VBOv;
    VBOt = _m.VBOt;
    EBOt = _m.EBOt;
    EBOl = _m.EBOl;
}

void Mesh::Draw(GLenum drawMode)
{
    glBindVertexArray(VAO);

    switch (drawMode)
    {
    case GL_TRIANGLES:
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOt);
        glDrawElements(drawMode, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, 0);
        break;

    case GL_LINE_LOOP:
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOl);
        glDrawElements(drawMode, static_cast<GLsizei>(vertices.size()), GL_UNSIGNED_INT, 0);
        break;

    default:
        SPDLOG_ERROR("Not a support draw mode");
        break;
    }
    glBindVertexArray(0);
}

} // namespace spe