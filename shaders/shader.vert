#version 460

layout(binding = 0) uniform UniformBufferObject {
    mat4 model;
    mat4 view;
    mat4 proj;
} ubo;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in uint inMeshIndex;

layout(location = 0)      out vec3 outNormal;
layout(location = 1) flat out uint outMeshIndex;
layout(location = 2)      out vec3 outWorldPos;

void main() {
    gl_Position = ubo.proj * ubo.view * ubo.model * vec4(inPosition, 1.0);
    outNormal = inNormal;
    outMeshIndex = inMeshIndex;
    outWorldPos = vec3((ubo.model * vec4(inPosition, 1.0)));
}
