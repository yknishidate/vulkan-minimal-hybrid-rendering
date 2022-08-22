
#include <set>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "swapchain.hpp"

using vkIL = vk::ImageLayout;
using vkA = vk::AccessFlagBits;
using vkBU = vk::BufferUsageFlagBits;
using vkMP = vk::MemoryPropertyFlagBits;
using vkDT = vk::DescriptorType;

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

struct Buffer
{
    vk::UniqueBuffer buffer;
    vk::UniqueDeviceMemory memory;
    vk::DeviceSize size;
    uint64_t deviceAddress;
    vk::DescriptorBufferInfo bufferInfo;
    void* mapped = nullptr;

    Buffer() = default;

    Buffer(vk::DeviceSize size, vk::BufferUsageFlags usage,
           vk::MemoryPropertyFlags properties)
        : size{ size }
    {
        buffer = Context::device.createBufferUnique({ {}, size, usage });

        vk::MemoryRequirements requirements = Context::device.getBufferMemoryRequirements(*buffer);
        uint32_t type = Context::findMemoryType(requirements.memoryTypeBits, properties);

        vk::MemoryAllocateInfo allocInfo{ requirements.size, type };
        if (usage & vk::BufferUsageFlagBits::eShaderDeviceAddress) {
            vk::MemoryAllocateFlagsInfo flagsInfo{ vk::MemoryAllocateFlagBits::eDeviceAddress };
            allocInfo.pNext = &flagsInfo;

            memory = Context::device.allocateMemoryUnique(allocInfo);
            Context::device.bindBufferMemory(*buffer, *memory, 0);

            vk::BufferDeviceAddressInfoKHR bufferDeviceAI{ *buffer };
            deviceAddress = Context::device.getBufferAddressKHR(&bufferDeviceAI);
        } else {
            memory = Context::device.allocateMemoryUnique(allocInfo);
            Context::device.bindBufferMemory(*buffer, *memory, 0);
        }
    }

    void copy(void* data)
    {
        if (!mapped) {
            mapped = Context::device.mapMemory(*memory, 0, size);
        }
        memcpy(mapped, data, size);
    }

    vk::WriteDescriptorSet createDescWrite(const vk::DescriptorSet& descSet,
                                           vk::DescriptorType type, uint32_t binding)
    {
        bufferInfo = vk::DescriptorBufferInfo{ *buffer, 0, size };
        vk::WriteDescriptorSet bufferWrite;
        bufferWrite.setDstSet(descSet);
        bufferWrite.setDescriptorType(type);
        bufferWrite.setDescriptorCount(1);
        bufferWrite.setDstBinding(binding);
        bufferWrite.setBufferInfo(bufferInfo);
        return bufferWrite;
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

    void load(const std::string& filepath)
    {
        std::ifstream file(filepath);
        std::string line;
        std::vector<glm::vec3> positions;
        std::vector<glm::vec3> normals;
        std::vector<std::pair<int, int>> verts;

        uint16_t currentMeshIndex = -1;
        while (std::getline(file, line)) {
            std::vector<std::string> list = split(line, ' ');
            if (list[0] == "v") {
                positions.push_back(glm::vec3(stof(list[1]), -stof(list[2]), stof(list[3])));
            }
            if (list[0] == "vn") {
                normals.push_back(glm::vec3(stof(list[1]), -stof(list[2]), stof(list[3])));
            }
            if (list[0] == "o") {
                currentMeshIndex++;
            }
            if (list[0] == "f") {
                for (int i = 1; i <= 3; i++) {
                    std::vector<std::string> vertAttrs = split(list[i], '/');
                    int posIndex = stoi(vertAttrs[0]) - 1;
                    int normalIndex = stoi(vertAttrs[2]) - 1;
                    std::pair vert{ posIndex, normalIndex };
                    auto itr = std::find(verts.begin(), verts.end(), vert);

                    if (itr == verts.end()) {
                        verts.push_back(vert);
                        Vertex v{ positions[posIndex], normals[normalIndex], currentMeshIndex };
                        vertices.push_back(v);
                        indices.push_back(vertices.size() - 1);
                    } else {
                        int index = std::distance(verts.begin(), itr);
                        indices.push_back(index);
                    }
                }
            }
        }
    }
};

struct AccelerationStructure
{
    vk::UniqueAccelerationStructureKHR handle;
    Buffer buffer;

    vk::AccelerationStructureTypeKHR type;
    uint32_t primitiveCount;
    vk::DeviceSize size;
    vk::AccelerationStructureBuildGeometryInfoKHR geometryInfo;
    uint64_t deviceAddress;
    vk::WriteDescriptorSetAccelerationStructureKHR asInfo;

    AccelerationStructure() = default;

    AccelerationStructure(vk::AccelerationStructureGeometryKHR geometry,
                          vk::AccelerationStructureTypeKHR type,
                          uint32_t primitiveCount)
    {
        this->type = type;
        this->primitiveCount = primitiveCount;

        geometryInfo.setType(type);
        geometryInfo.setFlags(vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
        geometryInfo.setGeometries(geometry);

        vk::AccelerationStructureBuildSizesInfoKHR buildSizesInfo =
            Context::device.getAccelerationStructureBuildSizesKHR(vk::AccelerationStructureBuildTypeKHR::eDevice, geometryInfo, primitiveCount);
        size = buildSizesInfo.accelerationStructureSize;
        buffer = Buffer{ size, vkBU::eAccelerationStructureStorageKHR | vkBU::eShaderDeviceAddress, vkMP::eDeviceLocal };

        vk::AccelerationStructureCreateInfoKHR createInfo;
        createInfo.setBuffer(*buffer.buffer);
        createInfo.setSize(size);
        createInfo.setType(type);
        handle = Context::device.createAccelerationStructureKHRUnique(createInfo);

        Buffer scratchBuffer{ size, vkBU::eStorageBuffer | vkBU::eShaderDeviceAddress, vkMP::eDeviceLocal };
        geometryInfo.setScratchData(scratchBuffer.deviceAddress);
        geometryInfo.setDstAccelerationStructure(*handle);

        vk::AccelerationStructureBuildRangeInfoKHR rangeInfo{ primitiveCount , 0, 0, 0 };
        Context::oneTimeSubmit([&](vk::CommandBuffer commandBuffer)
                               {
                                   commandBuffer.buildAccelerationStructuresKHR(geometryInfo, &rangeInfo);
                               });
    }

    vk::WriteDescriptorSet createDescWrite(vk::DescriptorSet& descSet, uint32_t binding)
    {
        asInfo = vk::WriteDescriptorSetAccelerationStructureKHR{ *handle };
        vk::WriteDescriptorSet asWrite;
        asWrite.setDstSet(descSet);
        asWrite.setDescriptorType(vkDT::eAccelerationStructureKHR);
        asWrite.setDescriptorCount(1);
        asWrite.setDstBinding(binding);
        asWrite.setPNext(&asInfo);
        return asWrite;
    }
};

struct UniformBufferObject
{
    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 proj;
};

class Application
{
public:
    void initVulkan()
    {
        createDescriptorSetLayout();
        createGraphicsPipeline();
        createVertexBuffer();
        createIndexBuffer();
        createUniformBuffers();
        createBottomLevelAS();
        createTopLevelAS();
        createDescriptorSets();
        createCommandBuffers();
    }

    void mainLoop()
    {
        while (!Window::shouldClose()) {
            Window::pollEvents();
            drawFrame();
        }
        Context::device.waitIdle();
        cleanup();
    }

    void setVertShaderPath(const std::string& path)
    {
        vertShaderPath = path;
    }

    void setFragShaderPath(const std::string& path)
    {
        fragShaderPath = path;
    }

    void setMesh(const Mesh& mesh)
    {
        this->vertices = mesh.vertices;
        this->indices = mesh.indices;
    }

private:
    Swapchain swapchain;

    vk::UniqueDescriptorSetLayout descriptorSetLayout;
    vk::UniqueDescriptorSet descriptorSet;

    std::vector<Vertex> vertices;
    std::vector<uint16_t> indices;
    std::string vertShaderPath;
    std::string fragShaderPath;
    vk::UniquePipelineLayout pipelineLayout;
    vk::UniquePipeline graphicsPipeline;

    Buffer vertexBuffer;
    Buffer indexBuffer;
    Buffer uniformBuffer;
    AccelerationStructure bottomLevelAS;
    AccelerationStructure topLevelAS;

    std::vector<vk::UniqueCommandBuffer> commandBuffers;

    void cleanup()
    {
        Window::terminate();
    }

    vk::UniqueShaderModule createShaderModule(const std::string& shaderPath)
    {
        auto code = readFile(shaderPath);
        vk::ShaderModuleCreateInfo createInfo;
        createInfo.setCodeSize(code.size());
        createInfo.setPCode(reinterpret_cast<const uint32_t*>(code.data()));
        return Context::device.createShaderModuleUnique(createInfo);
    }

    void createDescriptorSetLayout()
    {
        using vkSS = vk::ShaderStageFlagBits;
        std::vector<vk::DescriptorSetLayoutBinding> bindings;
        bindings.emplace_back(0, vkDT::eUniformBuffer, 1, vkSS::eVertex);
        bindings.emplace_back(1, vkDT::eAccelerationStructureKHR, 1, vkSS::eFragment);

        vk::DescriptorSetLayoutCreateInfo layoutInfo;
        layoutInfo.setBindings(bindings);
        descriptorSetLayout = Context::device.createDescriptorSetLayoutUnique(layoutInfo);
    }

    void createGraphicsPipeline()
    {
        vk::UniqueShaderModule vertShaderModule = createShaderModule(vertShaderPath);
        vk::UniqueShaderModule fragShaderModule = createShaderModule(fragShaderPath);
        vk::PipelineShaderStageCreateInfo vertShaderStageInfo;
        vertShaderStageInfo.setStage(vk::ShaderStageFlagBits::eVertex);
        vertShaderStageInfo.setModule(*vertShaderModule);
        vertShaderStageInfo.setPName("main");
        vk::PipelineShaderStageCreateInfo fragShaderStageInfo;
        fragShaderStageInfo.setStage(vk::ShaderStageFlagBits::eFragment);
        fragShaderStageInfo.setModule(*fragShaderModule);
        fragShaderStageInfo.setPName("main");
        std::array shaderStages{ vertShaderStageInfo, fragShaderStageInfo };

        auto bindingDescription = Vertex::getBindingDescription();
        auto attributeDescriptions = Vertex::getAttributeDescriptions();
        vk::PipelineVertexInputStateCreateInfo vertexInputInfo;
        vertexInputInfo.setVertexBindingDescriptions(bindingDescription);
        vertexInputInfo.setVertexAttributeDescriptions(attributeDescriptions);

        vk::PipelineInputAssemblyStateCreateInfo inputAssembly;
        inputAssembly.setTopology(vk::PrimitiveTopology::eTriangleList);

        float width = static_cast<float>(Window::getWidth());
        float height = static_cast<float>(Window::getHeight());
        vk::Viewport viewport{ 0.0f, 0.0f, width, height, 0.0f, 1.0f };
        vk::Rect2D scissor{ { 0, 0 }, swapchain.extent };
        vk::PipelineViewportStateCreateInfo viewportState{ {}, 1, &viewport, 1, &scissor };

        vk::PipelineRasterizationStateCreateInfo rasterizer;
        rasterizer.setDepthClampEnable(VK_FALSE);
        rasterizer.setRasterizerDiscardEnable(VK_FALSE);
        rasterizer.setPolygonMode(vk::PolygonMode::eFill);
        rasterizer.setCullMode(vk::CullModeFlagBits::eNone);
        rasterizer.setFrontFace(vk::FrontFace::eCounterClockwise);
        rasterizer.setDepthBiasEnable(VK_FALSE);
        rasterizer.setLineWidth(1.0f);

        vk::PipelineMultisampleStateCreateInfo multisampling;
        multisampling.setSampleShadingEnable(VK_FALSE);

        vk::PipelineDepthStencilStateCreateInfo depthStencil;
        depthStencil.setDepthTestEnable(VK_TRUE);
        depthStencil.setDepthWriteEnable(VK_TRUE);
        depthStencil.setDepthCompareOp(vk::CompareOp::eLess);
        depthStencil.setDepthBoundsTestEnable(VK_FALSE);
        depthStencil.setStencilTestEnable(VK_FALSE);

        using vkCC = vk::ColorComponentFlagBits;
        vk::PipelineColorBlendAttachmentState colorBlendAttachment;
        colorBlendAttachment.setBlendEnable(VK_FALSE);
        colorBlendAttachment.setColorWriteMask(vkCC::eR | vkCC::eG | vkCC::eB | vkCC::eA);

        vk::PipelineColorBlendStateCreateInfo colorBlending;
        colorBlending.setLogicOpEnable(VK_FALSE);
        colorBlending.setAttachments(colorBlendAttachment);

        vk::PipelineLayoutCreateInfo pipelineLayoutInfo;
        pipelineLayoutInfo.setSetLayouts(*descriptorSetLayout);
        pipelineLayout = Context::device.createPipelineLayoutUnique(pipelineLayoutInfo);

        vk::GraphicsPipelineCreateInfo pipelineInfo;
        pipelineInfo.setStages(shaderStages);
        pipelineInfo.setPVertexInputState(&vertexInputInfo);
        pipelineInfo.setPInputAssemblyState(&inputAssembly);
        pipelineInfo.setPViewportState(&viewportState);
        pipelineInfo.setPRasterizationState(&rasterizer);
        pipelineInfo.setPMultisampleState(&multisampling);
        pipelineInfo.setPDepthStencilState(&depthStencil);
        pipelineInfo.setPColorBlendState(&colorBlending);
        pipelineInfo.setLayout(*pipelineLayout);
        pipelineInfo.setRenderPass(*swapchain.renderPass);
        pipelineInfo.setSubpass(0);

        auto result = Context::device.createGraphicsPipelineUnique({}, pipelineInfo);
        if (result.result != vk::Result::eSuccess) {
            throw std::runtime_error("failed to create a pipeline!");
        }
        graphicsPipeline = std::move(result.value);
    }

    void createVertexBuffer()
    {
        vk::DeviceSize size = sizeof(Vertex) * vertices.size();
        vk::BufferUsageFlags usage{
            vkBU::eAccelerationStructureBuildInputReadOnlyKHR |
            vkBU::eStorageBuffer |
            vkBU::eShaderDeviceAddress |
            vkBU::eVertexBuffer
        };
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

    void createUniformBuffers()
    {
        uniformBuffer = Buffer{ sizeof(UniformBufferObject), vk::BufferUsageFlagBits::eUniformBuffer, vkMP::eHostVisible | vkMP::eHostCoherent };
    }

    void updateUniformBuffer()
    {
        using namespace std::chrono;
        static auto startTime = high_resolution_clock::now();
        auto currentTime = high_resolution_clock::now();
        float time = duration<float, seconds::period>(currentTime - startTime).count();

        float angle = time * glm::radians(60.0f);
        glm::mat4 rotate = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, -1.0f, 0.0f));
        glm::vec3 camPos = glm::vec3(rotate * glm::vec4(12.0f, -12.0f, 12.0f, 1.0f));
        float aspect = Window::getWidth() / (float)Window::getHeight();

        UniformBufferObject ubo;
        ubo.model = glm::mat4(1.0f);
        ubo.view = glm::lookAt(camPos, glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f));
        ubo.proj = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);
        ubo.proj[1][1] *= -1;

        uniformBuffer.copy(&ubo);
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

    void createTopLevelAS()
    {
        vk::TransformMatrixKHR transformMatrix = std::array{
            std::array{1.0f, 0.0f, 0.0f, 0.0f},
            std::array{0.0f, 1.0f, 0.0f, 0.0f},
            std::array{0.0f, 0.0f, 1.0f, 0.0f}
        };

        vk::AccelerationStructureInstanceKHR asInstance;
        asInstance.setTransform(transformMatrix);
        asInstance.setMask(0xFF);
        asInstance.setAccelerationStructureReference(bottomLevelAS.buffer.deviceAddress);
        asInstance.setFlags(vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable);

        Buffer instancesBuffer{ sizeof(vk::AccelerationStructureInstanceKHR),
                               vkBU::eAccelerationStructureBuildInputReadOnlyKHR |
                               vkBU::eShaderDeviceAddress,
                               vkMP::eHostVisible | vkMP::eHostCoherent };
        instancesBuffer.copy(&asInstance);

        vk::AccelerationStructureGeometryInstancesDataKHR instancesData;
        instancesData.setArrayOfPointers(false);
        instancesData.setData(instancesBuffer.deviceAddress);

        vk::AccelerationStructureGeometryKHR geometry;
        geometry.setGeometryType(vk::GeometryTypeKHR::eInstances);
        geometry.setGeometry({ instancesData });
        geometry.setFlags(vk::GeometryFlagBitsKHR::eOpaque);

        uint32_t primitiveCount = 1;
        topLevelAS = AccelerationStructure{ geometry,
                                            vk::AccelerationStructureTypeKHR::eTopLevel,
                                            primitiveCount };
    }

    void createDescriptorSets()
    {
        descriptorSet = Context::allocateDescSet(*descriptorSetLayout);

        std::vector descriptorWrites{
            uniformBuffer.createDescWrite(*descriptorSet, vkDT::eUniformBuffer, 0),
            topLevelAS.createDescWrite(*descriptorSet, 1)
        };
        Context::device.updateDescriptorSets(descriptorWrites, nullptr);
    }

    void createCommandBuffers()
    {
        vk::CommandBufferAllocateInfo allocInfo;
        allocInfo.setCommandPool(Context::commandPool);
        allocInfo.setCommandBufferCount(static_cast<uint32_t>(swapchain.framebuffers.size()));
        commandBuffers = Context::device.allocateCommandBuffersUnique(allocInfo);

        std::array<vk::ClearValue, 2> clearValues;
        clearValues[0].color = { std::array{0.0f, 0.0f, 0.0f, 1.0f} };
        clearValues[1].depthStencil = vk::ClearDepthStencilValue{ 1.0f, 0 };

        vk::RenderPassBeginInfo renderPassInfo;
        renderPassInfo.setRenderPass(*swapchain.renderPass);
        renderPassInfo.setRenderArea({ {0, 0}, {static_cast<uint32_t>(Window::getWidth()), static_cast<uint32_t>(Window::getHeight())} });
        renderPassInfo.setClearValues(clearValues);
        for (size_t i = 0; i < commandBuffers.size(); i++) {
            renderPassInfo.setFramebuffer(*swapchain.framebuffers[i]);
            commandBuffers[i]->begin(vk::CommandBufferBeginInfo{});
            commandBuffers[i]->beginRenderPass(renderPassInfo, vk::SubpassContents::eInline);
            commandBuffers[i]->bindPipeline(vk::PipelineBindPoint::eGraphics, *graphicsPipeline);

            vk::DeviceSize offsets{ 0 };
            commandBuffers[i]->bindVertexBuffers(0, *vertexBuffer.buffer, offsets);
            commandBuffers[i]->bindIndexBuffer(*indexBuffer.buffer, 0, vk::IndexType::eUint16);
            commandBuffers[i]->bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                                                  *pipelineLayout, 0, *descriptorSet, nullptr);
            commandBuffers[i]->drawIndexed(static_cast<uint32_t>(indices.size()), 1, 0, 0, 0);

            commandBuffers[i]->endRenderPass();
            commandBuffers[i]->end();
        }
    }

    void drawFrame()
    {
        uint32_t imageIndex = swapchain.acquireNextImage();
        updateUniformBuffer();
        swapchain.submit(*commandBuffers[imageIndex]);
        swapchain.present(imageIndex);
    }

    static std::vector<char> readFile(const std::string& filename)
    {
        std::ifstream file(filename, std::ios::ate | std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("failed to open file!");
        }
        size_t fileSize = file.tellg();
        std::vector<char> buffer(fileSize);
        file.seekg(0);
        file.read(buffer.data(), fileSize);
        file.close();
        return buffer;
    }
};

int main()
{
    try {
        Window::init(1000, 800);
        Context::init();

        {
            Mesh mesh;
            mesh.load("../assets/bunny_and_teapot.obj");

            Application app;
            app.setVertShaderPath("../shaders/spv/shader.vert.spv");
            app.setFragShaderPath("../shaders/spv/shader.frag.spv");
            app.setMesh(mesh);
            app.initVulkan();
            app.mainLoop();
        }

        Context::terminate();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
