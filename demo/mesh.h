#pragma once

#include "common.h"

namespace muli
{

class Mesh : NonCopyable
{
public:
    Mesh(std::vector<Vec3> vertices, std::vector<Vec2> texCoords, std::vector<int32> indices);
    ~Mesh();

    void Draw(GLenum drawMode = GL_TRIANGLES) const;

private:
    std::vector<Vec3> vertices;
    std::vector<Vec2> texCoords;
    std::vector<int32> indices;

    GLuint VAO;
    GLuint VBOv;
    GLuint VBOt;
    GLuint EBOt;
    GLuint EBOl;

    bool moved = false;
};

} // namespace muli