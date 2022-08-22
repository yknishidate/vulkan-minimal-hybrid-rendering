
#include "pipeline.hpp"

void updateMatrices(Matrices& matrices)
{
    static uint64_t frame = 0;
    static float aspect = Window::width / static_cast<float>(Window::height);

    float angle = frame * glm::radians(1.0f);
    glm::mat4 rotate = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, -1.0f, 0.0f));
    glm::vec3 camPos = glm::vec3(rotate * glm::vec4(12.0f, -12.0f, 12.0f, 1.0f));

    matrices.model = glm::mat4{ 1.0f };
    matrices.view = glm::lookAt(camPos, glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f));
    matrices.proj = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);
    matrices.proj[1][1] *= -1;

    frame++;
}

int main()
{
    try {
        Window::init(1000, 800);
        Context::init();

        {
            Swapchain swapchain;

            GraphicsPipeline pipeline{
                "../shaders/spv/shader.vert.spv",
                "../shaders/spv/shader.frag.spv",
                swapchain.extent, *swapchain.renderPass
            };

            Scene scene{ "../assets/bunny_and_teapot.obj" };

            std::vector descriptorWrites{ scene.topLevelAS.createDescWrite(1) };
            pipeline.updateDescriptorSets(descriptorWrites);

            std::vector commandBuffers = Context::allocateCommandBuffers(swapchain.imageCount);

            Matrices matrices;

            while (!Window::shouldClose()) {
                Window::pollEvents();
                updateMatrices(matrices);

                uint32_t imageIndex = swapchain.acquireNextImage();
                vk::CommandBuffer commandBuffer = *commandBuffers[imageIndex];
                commandBuffer.begin(vk::CommandBufferBeginInfo{});
                swapchain.beginRenderPass(commandBuffer, imageIndex);
                pipeline.bind(commandBuffer);
                pipeline.pushMatrices(commandBuffer, &matrices);
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
