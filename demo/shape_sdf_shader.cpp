#include "shape_sdf_shader.h"

namespace muli
{

namespace
{

const char* vertexShader =
#ifndef __EMSCRIPTEN__
    R"(#version 330 core

    layout (location = 0) in vec2 unitCoord;
    layout (location = 1) in vec4 localBounds;
    layout (location = 2) in vec4 transform0;
    layout (location = 3) in vec4 transform1;
    layout (location = 4) in vec4 capsule;
    layout (location = 5) in vec4 fillColor;
    layout (location = 6) in vec4 outlineColor;
    layout (location = 7) in vec4 scalarData;
    layout (location = 8) in ivec4 meta;

    out vec2 outLocalPos;
    out vec4 outFillColor;
    out vec4 outOutlineColor;
    out vec2 outCenter;
    out vec4 outCapsule;
    out float outRadius;
    flat out ivec4 outMeta;

    uniform mat4 view;
    uniform mat4 proj;

    void main()
    {
        vec2 localPos = mix(localBounds.xy, localBounds.zw, unitCoord);
        vec2 translation = transform0.xy;
        vec2 rotX = transform0.zw;
        vec2 rotY = transform1.xy;
        vec2 worldPos = translation + rotX * localPos.x + rotY * localPos.y;

        outLocalPos = localPos;
        outFillColor = fillColor;
        outOutlineColor = outlineColor;
        outCenter = transform1.zw;
        outCapsule = capsule;
        outRadius = scalarData.x;
        outMeta = meta;

        gl_Position = proj * view * vec4(worldPos, 0.0, 1.0);
    }
)";
#else
    R"(#version 300 es
    precision highp float;
    precision highp int;

    layout (location = 0) in vec2 unitCoord;
    layout (location = 1) in vec4 localBounds;
    layout (location = 2) in vec4 transform0;
    layout (location = 3) in vec4 transform1;
    layout (location = 4) in vec4 capsule;
    layout (location = 5) in vec4 fillColor;
    layout (location = 6) in vec4 outlineColor;
    layout (location = 7) in vec4 scalarData;
    layout (location = 8) in ivec4 meta;

    out vec2 outLocalPos;
    out vec4 outFillColor;
    out vec4 outOutlineColor;
    out vec2 outCenter;
    out vec4 outCapsule;
    out float outRadius;
    flat out ivec4 outMeta;

    uniform mat4 view;
    uniform mat4 proj;

    void main()
    {
        vec2 localPos = mix(localBounds.xy, localBounds.zw, unitCoord);
        vec2 translation = transform0.xy;
        vec2 rotX = transform0.zw;
        vec2 rotY = transform1.xy;
        vec2 worldPos = translation + rotX * localPos.x + rotY * localPos.y;

        outLocalPos = localPos;
        outFillColor = fillColor;
        outOutlineColor = outlineColor;
        outCenter = transform1.zw;
        outCapsule = capsule;
        outRadius = scalarData.x;
        outMeta = meta;

        gl_Position = proj * view * vec4(worldPos, 0.0, 1.0);
    }
)";
#endif

const char* fragmentShader =
#ifndef __EMSCRIPTEN__
    R"(#version 330 core

    const int maxPolygonVertices = 64;
    const int shapeTypeCircle = 0;
    const int shapeTypeCapsule = 1;
    const int shapeTypePolygon = 2;
    const int flagDrawFill = 1;
    const int flagDrawOutline = 2;

    in vec2 outLocalPos;
    in vec4 outFillColor;
    in vec4 outOutlineColor;
    in vec2 outCenter;
    in vec4 outCapsule;
    in float outRadius;
    flat in ivec4 outMeta;

    out vec4 fragColor;

    uniform vec2 pixelWorldSize;
    uniform sampler2D polygonVerticesTex;

    vec2 GetPolygonVertex(int index)
    {
        int packedIndex = outMeta.w + index;
        ivec2 texelCoord = ivec2(packedIndex % maxPolygonVertices, packedIndex / maxPolygonVertices);
        return texelFetch(polygonVerticesTex, texelCoord, 0).xy;
    }

    float sdCircle(vec2 p)
    {
        return length(p - outCenter) - outRadius;
    }

    float sdCapsule(vec2 p)
    {
        vec2 a = outCapsule.xy;
        vec2 b = outCapsule.zw;
        vec2 pa = p - a;
        vec2 ba = b - a;
        float denom = max(dot(ba, ba), 1e-6);
        float h = clamp(dot(pa, ba) / denom, 0.0, 1.0);
        return length(pa - ba * h) - outRadius;
    }

    float sdConvexPolygon(vec2 p)
    {
        if (outMeta.z <= 0)
        {
            return 1e30;
        }

        float maxSeparation = -1e30;
        float minDist2 = 1e30;

        for (int i = 0; i < maxPolygonVertices; ++i)
        {
            if (i >= outMeta.z)
            {
                break;
            }

            int next = i + 1;
            if (next >= outMeta.z)
            {
                next = 0;
            }

            vec2 a = GetPolygonVertex(i);
            vec2 b = GetPolygonVertex(next);
            vec2 edge = b - a;
            float edgeLen2 = max(dot(edge, edge), 1e-6);
            float h = clamp(dot(p - a, edge) / edgeLen2, 0.0, 1.0);
            vec2 closest = a + edge * h;
            vec2 delta = p - closest;
            minDist2 = min(minDist2, dot(delta, delta));

            vec2 normal = normalize(vec2(edge.y, -edge.x));
            maxSeparation = max(maxSeparation, dot(normal, p - a));
        }

        return maxSeparation > 0.0 ? sqrt(minDist2) : maxSeparation;
    }

    float ComputeDistance(vec2 p)
    {
        if (outMeta.x == shapeTypeCircle)
        {
            return sdCircle(p);
        }
        if (outMeta.x == shapeTypeCapsule)
        {
            return sdCapsule(p);
        }
        return sdConvexPolygon(p) - outRadius;
    }

    void main()
    {
        float distanceToShape = ComputeDistance(outLocalPos);
        float aa = max(fwidth(distanceToShape), 1e-6);
        float halfOutlineWidth = 0.5 * max(pixelWorldSize.x, pixelWorldSize.y);

        float fillAlpha = (outMeta.y & flagDrawFill) != 0 ? 1.0 - smoothstep(-aa, aa, distanceToShape) : 0.0;
        float outlineAlpha = 0.0;

        if ((outMeta.y & flagDrawOutline) != 0)
        {
            outlineAlpha = 1.0 - smoothstep(halfOutlineWidth - aa, halfOutlineWidth + aa, abs(distanceToShape));
        }

        float fillCoverage = outFillColor.a * fillAlpha;
        float outlineCoverage = outOutlineColor.a * outlineAlpha;
        float outAlpha = outlineCoverage + fillCoverage * (1.0 - outlineCoverage);

        if (outAlpha <= 0.0)
        {
            discard;
        }

        vec3 premul =
            outOutlineColor.rgb * outlineCoverage + outFillColor.rgb * fillCoverage * (1.0 - outlineCoverage);

        fragColor = vec4(premul / outAlpha, outAlpha);
    }
)";
#else
    R"(#version 300 es
    precision highp float;
    precision highp int;

    const int maxPolygonVertices = 64;
    const int shapeTypeCircle = 0;
    const int shapeTypeCapsule = 1;
    const int shapeTypePolygon = 2;
    const int flagDrawFill = 1;
    const int flagDrawOutline = 2;

    in vec2 outLocalPos;
    in vec4 outFillColor;
    in vec4 outOutlineColor;
    in vec2 outCenter;
    in vec4 outCapsule;
    in float outRadius;
    flat in ivec4 outMeta;

    out vec4 fragColor;

    uniform vec2 pixelWorldSize;
    uniform sampler2D polygonVerticesTex;

    vec2 GetPolygonVertex(int index)
    {
        int packedIndex = outMeta.w + index;
        ivec2 texelCoord = ivec2(packedIndex % maxPolygonVertices, packedIndex / maxPolygonVertices);
        return texelFetch(polygonVerticesTex, texelCoord, 0).xy;
    }

    float sdCircle(vec2 p)
    {
        return length(p - outCenter) - outRadius;
    }

    float sdCapsule(vec2 p)
    {
        vec2 a = outCapsule.xy;
        vec2 b = outCapsule.zw;
        vec2 pa = p - a;
        vec2 ba = b - a;
        float denom = max(dot(ba, ba), 1e-6);
        float h = clamp(dot(pa, ba) / denom, 0.0, 1.0);
        return length(pa - ba * h) - outRadius;
    }

    float sdConvexPolygon(vec2 p)
    {
        if (outMeta.z <= 0)
        {
            return 1e30;
        }

        float maxSeparation = -1e30;
        float minDist2 = 1e30;

        for (int i = 0; i < maxPolygonVertices; ++i)
        {
            if (i >= outMeta.z)
            {
                break;
            }

            int next = i + 1;
            if (next >= outMeta.z)
            {
                next = 0;
            }

            vec2 a = GetPolygonVertex(i);
            vec2 b = GetPolygonVertex(next);
            vec2 edge = b - a;
            float edgeLen2 = max(dot(edge, edge), 1e-6);
            float h = clamp(dot(p - a, edge) / edgeLen2, 0.0, 1.0);
            vec2 closest = a + edge * h;
            vec2 delta = p - closest;
            minDist2 = min(minDist2, dot(delta, delta));

            vec2 normal = normalize(vec2(edge.y, -edge.x));
            maxSeparation = max(maxSeparation, dot(normal, p - a));
        }

        return maxSeparation > 0.0 ? sqrt(minDist2) : maxSeparation;
    }

    float ComputeDistance(vec2 p)
    {
        if (outMeta.x == shapeTypeCircle)
        {
            return sdCircle(p);
        }
        if (outMeta.x == shapeTypeCapsule)
        {
            return sdCapsule(p);
        }
        return sdConvexPolygon(p) - outRadius;
    }

    void main()
    {
        float distanceToShape = ComputeDistance(outLocalPos);
        float aa = max(fwidth(distanceToShape), 1e-6);
        float halfOutlineWidth = 0.5 * max(pixelWorldSize.x, pixelWorldSize.y);

        float fillAlpha = (outMeta.y & flagDrawFill) != 0 ? 1.0 - smoothstep(-aa, aa, distanceToShape) : 0.0;
        float outlineAlpha = 0.0;

        if ((outMeta.y & flagDrawOutline) != 0)
        {
            outlineAlpha = 1.0 - smoothstep(halfOutlineWidth - aa, halfOutlineWidth + aa, abs(distanceToShape));
        }

        float fillCoverage = outFillColor.a * fillAlpha;
        float outlineCoverage = outOutlineColor.a * outlineAlpha;
        float outAlpha = outlineCoverage + fillCoverage * (1.0 - outlineCoverage);

        if (outAlpha <= 0.0)
        {
            discard;
        }

        vec3 premul =
            outOutlineColor.rgb * outlineCoverage + outFillColor.rgb * fillCoverage * (1.0 - outlineCoverage);

        fragColor = vec4(premul / outAlpha, outAlpha);
    }
)";
#endif

} // namespace

ShapeSdfShader::ShapeSdfShader()
    : Shader(vertexShader, fragmentShader)
{
    uniformMap.insert({ "view", glGetUniformLocation(shaderHandle, "view") });
    uniformMap.insert({ "proj", glGetUniformLocation(shaderHandle, "proj") });
    uniformMap.insert({ "pixelWorldSize", glGetUniformLocation(shaderHandle, "pixelWorldSize") });
    uniformMap.insert({ "polygonVerticesTex", glGetUniformLocation(shaderHandle, "polygonVerticesTex") });
}

} // namespace muli
