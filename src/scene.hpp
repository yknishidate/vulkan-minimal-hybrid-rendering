#pragma once

#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "accel.hpp"

struct Vertex
{
    glm::vec3 pos;
    glm::vec3 normal;
    uint16_t meshIndex;

    static vk::VertexInputBindingDescription getBindingDescription()
    {
        vk::VertexInputBindingDescription description;
        description.setBinding(0);
        description.setStride(sizeof(Vertex));
        description.setInputRate(vk::VertexInputRate::eVertex);
        return description;
    }

    static std::array<vk::VertexInputAttributeDescription, 3> getAttributeDescriptions()
    {
        std::array<vk::VertexInputAttributeDescription, 3> descriptions;
        descriptions[0].setBinding(0);
        descriptions[0].setLocation(0);
        descriptions[0].setFormat(vk::Format::eR32G32B32Sfloat);
        descriptions[0].setOffset(offsetof(Vertex, pos));

        descriptions[1].setBinding(0);
        descriptions[1].setLocation(1);
        descriptions[1].setFormat(vk::Format::eR32G32B32Sfloat);
        descriptions[1].setOffset(offsetof(Vertex, normal));

        descriptions[2].setBinding(0);
        descriptions[2].setLocation(2);
        descriptions[2].setFormat(vk::Format::eR16Uint);
        descriptions[2].setOffset(offsetof(Vertex, meshIndex));
        return descriptions;
    }
};


std::vector<std::string> split(std::string& str, char separator)
{
    std::vector<std::string> list;
    size_t offset = 0;
    while (1) {
        auto pos = str.find(separator, offset);
        if (pos == std::string::npos) {
            list.push_back(str.substr(offset));
            break;
        }
        list.push_back(str.substr(offset, pos - offset));
        offset = pos + 1;
    }
    return list;
}

struct Mesh
{
    std::vector<Vertex> vertices;
    std::vector<uint16_t> indices;
    Buffer vertexBuffer;
    Buffer indexBuffer;
    AccelerationStructure bottomLevelAS;

    void createVertexBuffer()
    {
        vk::DeviceSize size = sizeof(Vertex) * vertices.size();
        vk::BufferUsageFlags usage{
            vkBU::eAccelerationStructureBuildInputReadOnlyKHR |
            vkBU::eStorageBuffer |
            vkBU::eShaderDeviceAddress |
            vkBU::eVertexBuffer };
        vertexBuffer = Buffer{ size, usage, vkMP::eHostVisible | vkMP::eHostCoherent };
        vertexBuffer.copy(vertices.data());
    }

    void createIndexBuffer()
    {
        vk::DeviceSize size = sizeof(uint16_t) * indices.size();
        vk::BufferUsageFlags usage{
            vkBU::eAccelerationStructureBuildInputReadOnlyKHR |
            vkBU::eStorageBuffer |
            vkBU::eShaderDeviceAddress |
            vkBU::eIndexBuffer };
        indexBuffer = Buffer{ size, usage, vkMP::eHostVisible | vkMP::eHostCoherent };
        indexBuffer.copy(indices.data());
    }

    void createBottomLevelAS()
    {
        vk::AccelerationStructureGeometryTrianglesDataKHR triangleData;
        triangleData.setVertexFormat(vk::Format::eR32G32B32Sfloat);
        triangleData.setVertexData(vertexBuffer.deviceAddress);
        triangleData.setVertexStride(sizeof(Vertex));
        triangleData.setMaxVertex(vertices.size());
        triangleData.setIndexType(vk::IndexType::eUint16);
        triangleData.setIndexData(indexBuffer.deviceAddress);

        vk::AccelerationStructureGeometryKHR geometry;
        geometry.setGeometryType(vk::GeometryTypeKHR::eTriangles);
        geometry.setGeometry({ triangleData });
        geometry.setFlags(vk::GeometryFlagBitsKHR::eOpaque);

        uint32_t primitiveCount = indices.size() / 3;
        bottomLevelAS = AccelerationStructure{ geometry,
                                               vk::AccelerationStructureTypeKHR::eBottomLevel,
                                               primitiveCount };
    }

    void draw(vk::CommandBuffer commandBuffer)
    {
        vk::DeviceSize offsets{ 0 };
        commandBuffer.bindVertexBuffers(0, *vertexBuffer.buffer, offsets);
        commandBuffer.bindIndexBuffer(*indexBuffer.buffer, 0, vk::IndexType::eUint16);
        commandBuffer.drawIndexed(static_cast<uint32_t>(indices.size()), 1, 0, 0, 0);
    }
};

struct Scene
{
    std::vector<Mesh> meshes;
    AccelerationStructure topLevelAS;

    void build()
    {
        for (auto& mesh : meshes) {
            mesh.createVertexBuffer();
            mesh.createIndexBuffer();
            mesh.createBottomLevelAS();
        }
        createTopLevelAS();
    }

    void draw(vk::CommandBuffer commandBuffer)
    {
        for (auto& mesh : meshes) {
            mesh.draw(commandBuffer);
        }
    }

    void createTopLevelAS()
    {
        vk::TransformMatrixKHR transformMatrix = std::array{
            std::array{1.0f, 0.0f, 0.0f, 0.0f},
            std::array{0.0f, 1.0f, 0.0f, 0.0f},
            std::array{0.0f, 0.0f, 1.0f, 0.0f}
        };

        std::vector<vk::AccelerationStructureInstanceKHR> instances;
        for (auto& mesh : meshes) {
            vk::AccelerationStructureInstanceKHR instance;
            instance.setTransform(transformMatrix);
            instance.setMask(0xFF);
            instance.setAccelerationStructureReference(mesh.bottomLevelAS.buffer.deviceAddress);
            instance.setFlags(vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable);
            instances.push_back(instance);
        }

        Buffer instancesBuffer{ sizeof(vk::AccelerationStructureInstanceKHR) * instances.size(),
                               vkBU::eAccelerationStructureBuildInputReadOnlyKHR |
                               vkBU::eShaderDeviceAddress,
                               vkMP::eHostVisible | vkMP::eHostCoherent };
        instancesBuffer.copy(instances.data());

        vk::AccelerationStructureGeometryInstancesDataKHR instancesData;
        instancesData.setArrayOfPointers(false);
        instancesData.setData(instancesBuffer.deviceAddress);

        vk::AccelerationStructureGeometryKHR geometry;
        geometry.setGeometryType(vk::GeometryTypeKHR::eInstances);
        geometry.setGeometry({ instancesData });
        geometry.setFlags(vk::GeometryFlagBitsKHR::eOpaque);

        topLevelAS = AccelerationStructure{ geometry,
                                            vk::AccelerationStructureTypeKHR::eTopLevel,
                                            static_cast<uint32_t>(instances.size()) };
    }

    void load(const std::string& filepath)
    {
        std::ifstream file(filepath);
        std::string line;
        std::vector<glm::vec3> positions;
        std::vector<glm::vec3> normals;

        while (std::getline(file, line)) {
            std::vector<std::string> list = split(line, ' ');
            if (list[0] == "v") {
                positions.push_back(glm::vec3(stof(list[1]), -stof(list[2]), stof(list[3])));
            }
            if (list[0] == "vn") {
                normals.push_back(glm::vec3(stof(list[1]), -stof(list[2]), stof(list[3])));
            }
            if (list[0] == "o") {
                meshes.push_back({});
            }
            if (list[0] == "f") {
                for (int i = 1; i <= 3; i++) {
                    std::vector<std::string> vertAttrs = split(list[i], '/');
                    int posIndex = stoi(vertAttrs[0]) - 1;
                    int normalIndex = stoi(vertAttrs[2]) - 1;

                    Vertex v{ positions[posIndex], normals[normalIndex], static_cast<uint16_t>(meshes.size() - 1) };
                    meshes.back().vertices.push_back(v);
                    meshes.back().indices.push_back(meshes.back().vertices.size() - 1);
                }
            }
        }
    }
};
