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
    }

    uint32_t acquireNextImage(vk::Semaphore semaphore)
    {
        auto res = Context::device.acquireNextImageKHR(*swapchain, UINT64_MAX, semaphore);
        if (res.result != vk::Result::eSuccess) {
            throw std::runtime_error("failed to acquire next image!");
        }
        return res.value;
    }

    vk::Extent2D extent;
    uint32_t imageCount;

    vk::UniqueSwapchainKHR swapchain;
    std::vector<vk::Image> images;
    std::vector<vk::UniqueImageView> imageViews;
};
