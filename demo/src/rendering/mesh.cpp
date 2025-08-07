#include "mesh.h"

namespace muli
{

Mesh::Mesh(std::vector<Vec3> inVertices, std::vector<Vec2> inTexCoords, std::vector<int32> inIndices)
    : vertices{ std::move(inVertices) }
    , texCoords{ std::move(inTexCoords) }
    , indices{ std::move(inIndices) }
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

    // indices for triangles
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOt);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int32) * indices.size(), indices.data(), GL_STATIC_DRAW);

    std::vector<int32> indicesL(vertices.size());
    std::iota(indicesL.begin(), indicesL.end(), 0);

    // indices for outlines
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOl);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int32) * indicesL.size(), indicesL.data(), GL_STATIC_DRAW);
}

Mesh::~Mesh()
{
    if (moved)
    {
        return;
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBOv);
    glDeleteBuffers(1, &VBOt);
    glDeleteBuffers(1, &EBOt);
    glDeleteBuffers(1, &EBOl);
}

void Mesh::Draw(GLenum drawMode) const
{
    glBindVertexArray(VAO);

    switch (drawMode)
    {
    case GL_TRIANGLES:
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOt);
        glDrawElements(GL_TRIANGLES, GLsizei(indices.size()), GL_UNSIGNED_INT, 0);
        break;

    case GL_LINE_LOOP:
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOl);
        glDrawElements(GL_LINE_LOOP, GLsizei(vertices.size()), GL_UNSIGNED_INT, 0);
        break;

    case GL_POINTS:
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOl);
        glDrawElements(GL_POINTS, GLsizei(indices.size()), GL_UNSIGNED_INT, 0);
        break;

    default:
        MuliAssert(false && "Not a supported draw mode");
        break;
    }

    glBindVertexArray(0);
}

} // namespace muli