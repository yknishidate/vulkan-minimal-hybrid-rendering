#pragma once

#include "context.hpp"

struct Swapchain
{
    Swapchain()
    {
        // Create swapchain
        {
            extent.width = static_cast<uint32_t>(Window::getWidth());
            extent.height = static_cast<uint32_t>(Window::getHeight());
            imageCount = 3;

            vk::SwapchainCreateInfoKHR swapchainInfo{};
            swapchainInfo.setSurface(Context::surface);
            swapchainInfo.setMinImageCount(imageCount);
            swapchainInfo.setImageFormat(vk::Format::eB8G8R8A8Unorm);
            swapchainInfo.setImageColorSpace(vk::ColorSpaceKHR::eSrgbNonlinear);
            swapchainInfo.setImageExtent(extent);
            swapchainInfo.setImageArrayLayers(1);
            swapchainInfo.setImageUsage(vk::ImageUsageFlagBits::eColorAttachment);
            swapchainInfo.setPreTransform(vk::SurfaceTransformFlagBitsKHR::eIdentity);
            swapchainInfo.setPresentMode(vk::PresentModeKHR::eFifo);
            swapchainInfo.setClipped(true);
            swapchain = Context::device.createSwapchainKHRUnique(swapchainInfo);
        }

        // Create swapchain image views
        {
            images = Context::device.getSwapchainImagesKHR(*swapchain);
            imageViews.resize(imageCount);

            vk::ImageSubresourceRange subresourceRange{ vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1 };
            for (size_t i = 0; i < imageCount; i++) {
                vk::ImageViewCreateInfo createInfo;
                createInfo.setImage(images[i]);
                createInfo.setViewType(vk::ImageViewType::e2D);
                createInfo.setFormat(vk::Format::eB8G8R8A8Unorm);
                createInfo.setSubresourceRange(subresourceRange);
                imageViews[i] = Context::device.createImageViewUnique(createInfo);
            }
        }

        // Create render pass
        {
            vk::AttachmentDescription colorAttachment;
            colorAttachment.setFormat(vk::Format::eB8G8R8A8Unorm);
            colorAttachment.setLoadOp(vk::AttachmentLoadOp::eClear);
            colorAttachment.setStoreOp(vk::AttachmentStoreOp::eStore);
            colorAttachment.setStencilLoadOp(vk::AttachmentLoadOp::eDontCare);
            colorAttachment.setStencilStoreOp(vk::AttachmentStoreOp::eDontCare);
            colorAttachment.setFinalLayout(vk::ImageLayout::ePresentSrcKHR);

            vk::AttachmentDescription depthAttachment;
            depthAttachment.setFormat(vk::Format::eD32Sfloat);
            depthAttachment.setLoadOp(vk::AttachmentLoadOp::eClear);
            depthAttachment.setStoreOp(vk::AttachmentStoreOp::eDontCare);
            depthAttachment.setStencilLoadOp(vk::AttachmentLoadOp::eDontCare);
            depthAttachment.setStencilStoreOp(vk::AttachmentStoreOp::eDontCare);
            depthAttachment.setInitialLayout(vk::ImageLayout::eUndefined);
            depthAttachment.setFinalLayout(vk::ImageLayout::eDepthStencilAttachmentOptimal);

            vk::AttachmentReference colorAttachmentRef;
            colorAttachmentRef.setAttachment(0);
            colorAttachmentRef.setLayout(vk::ImageLayout::eColorAttachmentOptimal);

            vk::AttachmentReference depthAttachmentRef;
            depthAttachmentRef.setAttachment(1);
            depthAttachmentRef.setLayout(vk::ImageLayout::eDepthStencilAttachmentOptimal);

            vk::SubpassDescription subpass;
            subpass.setColorAttachments(colorAttachmentRef);
            subpass.setPDepthStencilAttachment(&depthAttachmentRef);

            using vkPS = vk::PipelineStageFlagBits;
            using vkA = vk::AccessFlagBits;
            vk::SubpassDependency dependency;
            dependency.setSrcSubpass(VK_SUBPASS_EXTERNAL);
            dependency.setDstSubpass(0);
            dependency.setSrcStageMask(vkPS::eColorAttachmentOutput | vkPS::eEarlyFragmentTests);
            dependency.setDstStageMask(vkPS::eColorAttachmentOutput | vkPS::eEarlyFragmentTests);
            dependency.setDstAccessMask(vkA::eColorAttachmentWrite | vkA::eDepthStencilAttachmentWrite);

            std::array attachments{ colorAttachment, depthAttachment };
            vk::RenderPassCreateInfo renderPassInfo({}, attachments, subpass, dependency);
            renderPass = Context::device.createRenderPassUnique(renderPassInfo);
        }

        // Create depth image
        {
            vk::ImageCreateInfo createInfo;
            createInfo.setImageType(vk::ImageType::e2D);
            createInfo.setExtent({ extent.width, extent.height, 1 });
            createInfo.setMipLevels(1);
            createInfo.setArrayLayers(1);
            createInfo.setFormat(vk::Format::eD32Sfloat);
            createInfo.setTiling(vk::ImageTiling::eOptimal);
            createInfo.setUsage(vk::ImageUsageFlagBits::eDepthStencilAttachment);
            depthImage = Context::device.createImageUnique(createInfo);
        }

        // Allocate memory
        {
            vk::MemoryRequirements requirements = Context::device.getImageMemoryRequirements(*depthImage);
            uint32_t memoryTypeIndex = Context::findMemoryType(requirements.memoryTypeBits, vk::MemoryPropertyFlagBits::eDeviceLocal);
            depthImageMemory = Context::device.allocateMemoryUnique({ requirements.size, memoryTypeIndex });
            Context::device.bindImageMemory(*depthImage, *depthImageMemory, 0);
        }

        // Create depth image view
        {
            vk::ImageViewCreateInfo viewInfo;
            viewInfo.setImage(*depthImage);
            viewInfo.setViewType(vk::ImageViewType::e2D);
            viewInfo.setFormat(vk::Format::eD32Sfloat);
            viewInfo.setSubresourceRange({ vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1 });
            depthImageView = Context::device.createImageViewUnique(viewInfo);
        }

        // Create framebuffers
        {
            framebuffers.reserve(imageCount);
            for (auto const& view : imageViews) {
                std::array attachments{ *view, *depthImageView };
                vk::FramebufferCreateInfo framebufferInfo;
                framebufferInfo.setRenderPass(*renderPass);
                framebufferInfo.setAttachments(attachments);
                framebufferInfo.setWidth(static_cast<uint32_t>(Window::getWidth()));
                framebufferInfo.setHeight(static_cast<uint32_t>(Window::getHeight()));
                framebufferInfo.setLayers(1);
                framebuffers.push_back(Context::device.createFramebufferUnique(framebufferInfo));
            }
        }

        // Create sync objects
        {
            imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
            renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
            inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);

            for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
                imageAvailableSemaphores[i] = Context::device.createSemaphoreUnique({});
                renderFinishedSemaphores[i] = Context::device.createSemaphoreUnique({});
                inFlightFences[i] = Context::device.createFenceUnique({ vk::FenceCreateFlagBits::eSignaled });
            }
        }
    }

    uint32_t acquireNextImage()
    {
        Context::device.waitForFences(*inFlightFences[currentFrame], true, UINT64_MAX);
        Context::device.resetFences(*inFlightFences[currentFrame]);
        auto result = Context::device.acquireNextImageKHR(*swapchain, UINT64_MAX, *imageAvailableSemaphores[currentFrame]);
        if (result.result != vk::Result::eSuccess) {
            throw std::runtime_error("failed to acquire next image!");
        }
        return result.value;
    }

    void submit(vk::CommandBuffer commandBuffer)
    {
        vk::PipelineStageFlags waitStage{ vk::PipelineStageFlagBits::eColorAttachmentOutput };

        vk::SubmitInfo submitInfo;
        submitInfo.setWaitSemaphores(*imageAvailableSemaphores[currentFrame]);
        submitInfo.setWaitDstStageMask(waitStage);
        submitInfo.setCommandBuffers(commandBuffer);
        submitInfo.setSignalSemaphores(*renderFinishedSemaphores[currentFrame]);

        Context::queue.submit(submitInfo, *inFlightFences[currentFrame]);
    }

    void present(uint32_t imageIndex)
    {
        vk::PresentInfoKHR presentInfo{ *renderFinishedSemaphores[currentFrame], *swapchain, imageIndex };
        Context::queue.presentKHR(presentInfo);

        currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
    }

    const int MAX_FRAMES_IN_FLIGHT = 2;

    vk::Extent2D extent;
    uint32_t imageCount;

    vk::UniqueSwapchainKHR swapchain;
    std::vector<vk::Image> images;
    std::vector<vk::UniqueImageView> imageViews;
    vk::UniqueRenderPass renderPass;
    std::vector<vk::UniqueFramebuffer> framebuffers;

    vk::UniqueImage depthImage;
    vk::UniqueImageView depthImageView;
    vk::UniqueDeviceMemory depthImageMemory;

    std::vector<vk::UniqueSemaphore> imageAvailableSemaphores;
    std::vector<vk::UniqueSemaphore> renderFinishedSemaphores;
    std::vector<vk::UniqueFence> inFlightFences;
    size_t currentFrame = 0;
};
