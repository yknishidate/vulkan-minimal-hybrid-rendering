
#include "scene.hpp"
#include "swapchain.hpp"

using vkBU = vk::BufferUsageFlagBits;
using vkMP = vk::MemoryPropertyFlagBits;
using vkDT = vk::DescriptorType;

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
        createUniformBuffers();
        scene.build();
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

    void load(const std::string& filepath)
    {
        scene.load(filepath);
    }

private:
    Swapchain swapchain;

    vk::UniqueDescriptorSetLayout descriptorSetLayout;
    vk::UniqueDescriptorSet descriptorSet;

    Scene scene;
    std::string vertShaderPath;
    std::string fragShaderPath;
    vk::UniquePipelineLayout pipelineLayout;
    vk::UniquePipeline graphicsPipeline;

    Buffer uniformBuffer;

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

        float width = static_cast<float>(Window::width);
        float height = static_cast<float>(Window::height);
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

    void createUniformBuffers()
    {
        uniformBuffer = Buffer{ sizeof(UniformBufferObject), vk::BufferUsageFlagBits::eUniformBuffer, vkMP::eHostVisible | vkMP::eHostCoherent };
    }

    void updateUniformBuffer()
    {
        static uint64_t frame = 0;

        float angle = frame * glm::radians(1.0f);
        glm::mat4 rotate = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, -1.0f, 0.0f));
        glm::vec3 camPos = glm::vec3(rotate * glm::vec4(12.0f, -12.0f, 12.0f, 1.0f));
        float aspect = Window::width / static_cast<float>(Window::height);

        UniformBufferObject ubo;
        ubo.model = glm::mat4{ 1.0f };
        ubo.view = glm::lookAt(camPos, glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f));
        ubo.proj = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);
        ubo.proj[1][1] *= -1;

        uniformBuffer.copy(&ubo);
        frame++;
    }

    void createDescriptorSets()
    {
        descriptorSet = Context::allocateDescSet(*descriptorSetLayout);

        std::vector descriptorWrites{
            uniformBuffer.createDescWrite(*descriptorSet, vkDT::eUniformBuffer, 0),
            scene.topLevelAS.createDescWrite(*descriptorSet, 1)
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
        renderPassInfo.setRenderArea({ {0, 0}, swapchain.extent });
        renderPassInfo.setClearValues(clearValues);
        for (size_t i = 0; i < commandBuffers.size(); i++) {
            renderPassInfo.setFramebuffer(*swapchain.framebuffers[i]);
            commandBuffers[i]->begin(vk::CommandBufferBeginInfo{});
            commandBuffers[i]->beginRenderPass(renderPassInfo, vk::SubpassContents::eInline);
            commandBuffers[i]->bindPipeline(vk::PipelineBindPoint::eGraphics, *graphicsPipeline);

            //vk::DeviceSize offsets{ 0 };
            //commandBuffers[i]->bindVertexBuffers(0, *vertexBuffer.buffer, offsets);
            //commandBuffers[i]->bindIndexBuffer(*indexBuffer.buffer, 0, vk::IndexType::eUint16);
            commandBuffers[i]->bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                                                  *pipelineLayout, 0, *descriptorSet, nullptr);
            scene.draw(*commandBuffers[i]);
            //commandBuffers[i]->drawIndexed(static_cast<uint32_t>(indices.size()), 1, 0, 0, 0);

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
            Application app;
            app.load("../assets/bunny_and_teapot.obj");
            app.setVertShaderPath("../shaders/spv/shader.vert.spv");
            app.setFragShaderPath("../shaders/spv/shader.frag.spv");
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
