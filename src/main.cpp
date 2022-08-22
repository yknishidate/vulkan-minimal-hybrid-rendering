
#include "pipeline.hpp"

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
        graphicsPipeline = GraphicsPipeline{
            "../shaders/spv/shader.vert.spv",
            "../shaders/spv/shader.frag.spv",
            swapchain.extent,
            *swapchain.renderPass
        };

        createUniformBuffers();
        scene.build();

        std::vector descriptorWrites{
            uniformBuffer.createDescWrite(vkDT::eUniformBuffer, 0),
            scene.topLevelAS.createDescWrite(1)
        };

        graphicsPipeline.updateDescriptorSets(descriptorWrites);

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
    GraphicsPipeline graphicsPipeline;

    Scene scene;
    std::string vertShaderPath;
    std::string fragShaderPath;

    Buffer uniformBuffer;

    std::vector<vk::UniqueCommandBuffer> commandBuffers;

    void cleanup()
    {
        Window::terminate();
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

            graphicsPipeline.bind(*commandBuffers[i]);
            scene.draw(*commandBuffers[i]);
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
