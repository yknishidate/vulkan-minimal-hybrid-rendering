#pragma once

#include <fstream>

#include "scene.hpp"
#include "swapchain.hpp"

inline std::vector<char> readFile(const std::string& filename)
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

inline vk::UniqueShaderModule createShaderModule(const std::string& shaderPath)
{
    auto code = readFile(shaderPath);
    vk::ShaderModuleCreateInfo createInfo;
    createInfo.setCodeSize(code.size());
    createInfo.setPCode(reinterpret_cast<const uint32_t*>(code.data()));
    return Context::device.createShaderModuleUnique(createInfo);
}

struct Matrices
{
    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 proj;
};

struct GraphicsPipeline
{
    using vkDT = vk::DescriptorType;
    using vkSS = vk::ShaderStageFlagBits;

    GraphicsPipeline(const std::string& vertShaderPath,
                     const std::string& fragShaderPath,
                     vk::Extent2D extent)
    {
        // Create descriptor set layout
        {
            vk::DescriptorSetLayoutBinding binding{ 0, vkDT::eAccelerationStructureKHR, 1, vkSS::eFragment };

            vk::DescriptorSetLayoutCreateInfo layoutInfo;
            layoutInfo.setBindings(binding);
            descriptorSetLayout = Context::device.createDescriptorSetLayoutUnique(layoutInfo);
        }

        // Allocate descriptor set
        {
            descriptorSet = Context::allocateDescSet(*descriptorSetLayout);
        }

        // Create pipeline layout
        {
            vk::PushConstantRange pushRange;
            pushRange.setOffset(0);
            pushRange.setSize(sizeof(Matrices));
            pushRange.setStageFlags(vkSS::eVertex);

            vk::PipelineLayoutCreateInfo pipelineLayoutInfo;
            pipelineLayoutInfo.setSetLayouts(*descriptorSetLayout);
            pipelineLayoutInfo.setPushConstantRanges(pushRange);
            pipelineLayout = Context::device.createPipelineLayoutUnique(pipelineLayoutInfo);
        }

        // Create graphics pipeline
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

            auto width = static_cast<float>(Window::width);
            auto height = static_cast<float>(Window::height);
            vk::Viewport viewport{ 0.0f, 0.0f, width, height, 0.0f, 1.0f };
            vk::Rect2D scissor{ { 0, 0 }, extent };
            vk::PipelineViewportStateCreateInfo viewportState{ {}, 1, &viewport, 1, &scissor };

            vk::PipelineRasterizationStateCreateInfo rasterization;
            rasterization.setDepthClampEnable(VK_FALSE);
            rasterization.setRasterizerDiscardEnable(VK_FALSE);
            rasterization.setPolygonMode(vk::PolygonMode::eFill);
            rasterization.setCullMode(vk::CullModeFlagBits::eNone);
            rasterization.setFrontFace(vk::FrontFace::eCounterClockwise);
            rasterization.setDepthBiasEnable(VK_FALSE);
            rasterization.setLineWidth(1.0f);

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

            vk::Format colorFormat = vk::Format::eB8G8R8A8Unorm;
            vk::Format depthFormat = vk::Format::eD32Sfloat;
            vk::PipelineRenderingCreateInfo renderingInfo;
            renderingInfo.setColorAttachmentCount(1);
            renderingInfo.setColorAttachmentFormats(colorFormat);
            renderingInfo.setDepthAttachmentFormat(depthFormat);

            vk::GraphicsPipelineCreateInfo pipelineInfo;
            pipelineInfo.setStages(shaderStages);
            pipelineInfo.setPVertexInputState(&vertexInputInfo);
            pipelineInfo.setPInputAssemblyState(&inputAssembly);
            pipelineInfo.setPViewportState(&viewportState);
            pipelineInfo.setPRasterizationState(&rasterization);
            pipelineInfo.setPMultisampleState(&multisampling);
            pipelineInfo.setPDepthStencilState(&depthStencil);
            pipelineInfo.setPColorBlendState(&colorBlending);
            pipelineInfo.setLayout(*pipelineLayout);
            pipelineInfo.setSubpass(0);
            pipelineInfo.setPNext(&renderingInfo);

            auto result = Context::device.createGraphicsPipelineUnique({}, pipelineInfo);
            if (result.result != vk::Result::eSuccess) {
                throw std::runtime_error("failed to create a pipeline!");
            }
            pipeline = std::move(result.value);
        }
    }

    void pushMatrices(vk::CommandBuffer commandBuffer, const Matrices* matrices)
    {
        commandBuffer.pushConstants(*pipelineLayout, vk::ShaderStageFlagBits::eVertex, 0, sizeof(Matrices), matrices);
    }

    void updateDescriptorSets(std::vector<vk::WriteDescriptorSet> writes)
    {
        for (auto& write : writes) {
            write.setDstSet(*descriptorSet);
        }
        Context::device.updateDescriptorSets(writes, nullptr);
    }

    void bind(vk::CommandBuffer commandBuffer)
    {
        commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, *pipeline);
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                                         *pipelineLayout, 0, *descriptorSet, nullptr);
    }

    vk::UniqueDescriptorSetLayout descriptorSetLayout;
    vk::UniqueDescriptorSet descriptorSet;

    vk::UniquePipelineLayout pipelineLayout;
    vk::UniquePipeline pipeline;
};
