#version 460
#extension GL_EXT_ray_query : enable

layout(binding = 0) uniform accelerationStructureEXT topLevelAS;

layout(location = 0)      in vec3 inNormal;
layout(location = 1) flat in uint inMeshIndex;
layout(location = 2)      in vec3 inWorldPos;

layout(location = 0) out vec4 outColor;

void main() {
    vec3 colors[3] = {vec3(0.1, 0.1, 1.0), vec3(1.0), vec3(1.0, 0.1, 0.1)};
    vec3 lightDir = normalize(vec3(1, -1, 2));
    float lighting = max(0.0, dot(lightDir, normalize(inNormal)));
    outColor = vec4(colors[inMeshIndex] * lighting, 1.0);

    rayQueryEXT rayQuery;
    rayQueryInitializeEXT(
        rayQuery,
        topLevelAS,
        gl_RayFlagsTerminateOnFirstHitEXT, // rayFlags
        0xFF,                              // cullMask
        inWorldPos,                        // origin
        0.01,                              // tMin
        lightDir,                          // direction
        100.0                              // tMax
    );
    while (rayQueryProceedEXT(rayQuery)) { }
    if (rayQueryGetIntersectionTypeEXT(rayQuery, true) == gl_RayQueryCommittedIntersectionTriangleEXT ) {
        outColor *= 0.2;
    }
}
