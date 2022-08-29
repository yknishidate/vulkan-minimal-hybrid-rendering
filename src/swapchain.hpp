#pragma once

#include "context.hpp"

struct Swapchain
{
    Swapchain()
    {
        // Create swapchain
        {
            extent.width = static_cast<uint32_t>(Window::width);
            extent.height = static_cast<uint32_t>(Window::height);

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

            vk::ImageSubresourceRange subresourceRange{ vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1 };
            for (size_t i = 0; i < imageCount; i++) {
                vk::ImageViewCreateInfo createInfo;
                createInfo.setImage(images[i]);
                createInfo.setViewType(vk::ImageViewType::e2D);
                createInfo.setFormat(vk::Format::eB8G8R8A8Unorm);
                createInfo.setSubresourceRange(subresourceRange);
                imageViews.push_back(Context::device.createImageViewUnique(createInfo));
            }
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

        // Create sync objects
        {
            for (size_t i = 0; i < maxFramesCount; i++) {
                imageAvailableSemaphores.push_back(Context::device.createSemaphoreUnique({}));
                renderFinishedSemaphores.push_back(Context::device.createSemaphoreUnique({}));
                inFlightFences.push_back(Context::device.createFenceUnique({ vk::FenceCreateFlagBits::eSignaled }));
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

        currentFrame = (currentFrame + 1) % maxFramesCount;
    }

    void beginRendering(vk::CommandBuffer commandBuffer, uint32_t imageIndex)
    {
        vk::ClearValue clearColorValue;
        clearColorValue.setColor(vk::ClearColorValue{ std::array{0.0f, 0.0f, 0.0f, 1.0f} });

        vk::RenderingAttachmentInfo colorAttachment;
        colorAttachment.setImageView(*imageViews[imageIndex]);
        colorAttachment.setImageLayout(vk::ImageLayout::eAttachmentOptimal);
        colorAttachment.setClearValue(clearColorValue);
        colorAttachment.setLoadOp(vk::AttachmentLoadOp::eClear);
        colorAttachment.setStoreOp(vk::AttachmentStoreOp::eStore);

        vk::ClearValue clearDepthStencilValue;
        clearDepthStencilValue.setDepthStencil(vk::ClearDepthStencilValue{ 1.0f, 0 });

        vk::RenderingAttachmentInfo depthStencilAttachment;
        depthStencilAttachment.setImageView(*depthImageView);
        depthStencilAttachment.setImageLayout(vk::ImageLayout::eAttachmentOptimal);
        depthStencilAttachment.setClearValue(clearDepthStencilValue);
        depthStencilAttachment.setLoadOp(vk::AttachmentLoadOp::eClear);
        depthStencilAttachment.setStoreOp(vk::AttachmentStoreOp::eStore);

        vk::RenderingInfo renderingInfo;
        renderingInfo.setRenderArea({ {0, 0}, extent });
        renderingInfo.setLayerCount(1);
        renderingInfo.setColorAttachments(colorAttachment);
        renderingInfo.setPDepthAttachment(&depthStencilAttachment);

        commandBuffer.beginRendering(renderingInfo);
    }

    void endRendering(vk::CommandBuffer commandBuffer, uint32_t imageIndex)
    {
        commandBuffer.endRendering();

        vk::ImageMemoryBarrier imageMemoryBarrier;
        imageMemoryBarrier.setSrcAccessMask(vk::AccessFlagBits::eColorAttachmentWrite);
        imageMemoryBarrier.setOldLayout(vk::ImageLayout::eUndefined);
        imageMemoryBarrier.setNewLayout(vk::ImageLayout::ePresentSrcKHR);
        imageMemoryBarrier.setImage(images[imageIndex]);
        imageMemoryBarrier.setSubresourceRange({ vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1 });

        commandBuffer.pipelineBarrier(
            vk::PipelineStageFlagBits::eColorAttachmentOutput,
            vk::PipelineStageFlagBits::eBottomOfPipe,
            {}, {}, {}, imageMemoryBarrier);
    }

    const uint32_t maxFramesCount = 2;
    const uint32_t imageCount = 3;

    vk::Extent2D extent;

    vk::UniqueSwapchainKHR swapchain;
    std::vector<vk::Image> images;
    std::vector<vk::UniqueImageView> imageViews;

    vk::UniqueImage depthImage;
    vk::UniqueImageView depthImageView;
    vk::UniqueDeviceMemory depthImageMemory;

    std::vector<vk::UniqueSemaphore> imageAvailableSemaphores;
    std::vector<vk::UniqueSemaphore> renderFinishedSemaphores;
    std::vector<vk::UniqueFence> inFlightFences;
    size_t currentFrame = 0;
};
