
#include "pipeline.hpp"

struct UniformBufferObject
{
    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 proj;

    void update()
    {
        static uint64_t frame = 0;

        float angle = frame * glm::radians(1.0f);
        glm::mat4 rotate = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, -1.0f, 0.0f));
        glm::vec3 camPos = glm::vec3(rotate * glm::vec4(12.0f, -12.0f, 12.0f, 1.0f));
        float aspect = Window::width / static_cast<float>(Window::height);

        model = glm::mat4{ 1.0f };
        view = glm::lookAt(camPos, glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f));
        proj = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);
        proj[1][1] *= -1;

        frame++;
    }
};

int main()
{
    try {
        Window::init(1000, 800);
        Context::init();

        {
            Swapchain swapchain;

            GraphicsPipeline pipeline{
                "../shaders/spv/shader.vert.spv", "../shaders/spv/shader.frag.spv",
                swapchain.extent, *swapchain.renderPass
            };

            Scene scene{ "../assets/bunny_and_teapot.obj" };

            Buffer uniformBuffer{
                sizeof(UniformBufferObject),
                vk::BufferUsageFlagBits::eUniformBuffer,
                vkMP::eHostVisible | vkMP::eHostCoherent
            };

            std::vector descriptorWrites{
                uniformBuffer.createDescWrite(vkDT::eUniformBuffer, 0),
                scene.topLevelAS.createDescWrite(1)
            };
            pipeline.updateDescriptorSets(descriptorWrites);

            std::vector commandBuffers = Context::allocateCommandBuffers(swapchain.imageCount);

            UniformBufferObject ubo;

            while (!Window::shouldClose()) {
                Window::pollEvents();
                uint32_t imageIndex = swapchain.acquireNextImage();

                ubo.update();
                uniformBuffer.copy(&ubo);

                vk::CommandBuffer commandBuffer = *commandBuffers[imageIndex];
                commandBuffer.begin(vk::CommandBufferBeginInfo{});
                swapchain.beginRenderPass(commandBuffer, imageIndex);
                pipeline.bind(commandBuffer);
                scene.draw(commandBuffer);
                swapchain.endRenderPass(commandBuffer);
                commandBuffer.end();

                swapchain.submit(commandBuffer);
                swapchain.present(imageIndex);
            }
            Context::device.waitIdle();
        }

        Context::terminate();
        Window::terminate();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
