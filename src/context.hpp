#pragma once

#include <string>
#include <iostream>
#include <functional>

#define VULKAN_HPP_DISPATCH_LOADER_DYNAMIC 1
#include <vulkan/vulkan.hpp>

#include "window.hpp"

VULKAN_HPP_DEFAULT_DISPATCH_LOADER_DYNAMIC_STORAGE

struct Context
{
    static VKAPI_ATTR VkBool32 VKAPI_CALL debugUtilsMessengerCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
        VkDebugUtilsMessageTypeFlagsEXT messageTypes,
        VkDebugUtilsMessengerCallbackDataEXT const* pCallbackData, void* pUserData)
    {
        std::cerr << pCallbackData->pMessage << "\n\n";
        return VK_FALSE;
    }

    static void init()
    {
        // Create instance
        {
            std::vector extensions = Window::getRequiredInstanceExtensions();
            extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);

            std::vector layers{ "VK_LAYER_KHRONOS_validation", "VK_LAYER_LUNARG_monitor" };

            static vk::DynamicLoader dl;
            auto vkGetInstanceProcAddr = dl.getProcAddress<PFN_vkGetInstanceProcAddr>("vkGetInstanceProcAddr");
            VULKAN_HPP_DEFAULT_DISPATCHER.init(vkGetInstanceProcAddr);

            vk::ApplicationInfo appInfo;
            appInfo.setApiVersion(VK_API_VERSION_1_3);

            instance = vk::createInstance({ {}, &appInfo, layers, extensions });
            VULKAN_HPP_DEFAULT_DISPATCHER.init(instance);
            physicalDevice = instance.enumeratePhysicalDevices().front();
        }

        // Create debug messenger
        {
            vk::DebugUtilsMessengerCreateInfoEXT messengerInfo;
            messengerInfo.setMessageSeverity(vk::DebugUtilsMessageSeverityFlagBitsEXT::eError);
            messengerInfo.setMessageType(vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation);
            messengerInfo.setPfnUserCallback(&debugUtilsMessengerCallback);
            messenger = instance.createDebugUtilsMessengerEXT(messengerInfo);
        }

        // Create surface
        {
            surface = Window::createSurface(instance);
        }

        // Find queue family
        {
            auto queueFamilies = physicalDevice.getQueueFamilyProperties();
            for (int i = 0; i < queueFamilies.size(); i++) {
                auto supportGraphics = queueFamilies[i].queueFlags & vk::QueueFlagBits::eGraphics;
                auto supportPresent = physicalDevice.getSurfaceSupportKHR(i, surface);
                if (supportGraphics && supportPresent) {
                    queueFamily = i;
                }
            }
        }

        // Create device
        {
            float queuePriority = 1.0f;
            vk::DeviceQueueCreateInfo queueCreateInfo{ {}, queueFamily, 1, &queuePriority };

            const std::vector deviceExtensions{
                VK_KHR_SWAPCHAIN_EXTENSION_NAME,
                VK_KHR_MAINTENANCE3_EXTENSION_NAME,
                VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
                VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME,
                VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
                VK_KHR_RAY_QUERY_EXTENSION_NAME,
            };

            vk::PhysicalDeviceFeatures deviceFeatures;
            vk::DeviceCreateInfo createInfo{ {}, queueCreateInfo, {}, deviceExtensions, &deviceFeatures };
            vk::StructureChain createInfoChain{ createInfo,
                vk::PhysicalDeviceBufferDeviceAddressFeatures{true},
                vk::PhysicalDeviceAccelerationStructureFeaturesKHR{true},
                vk::PhysicalDeviceRayQueryFeaturesKHR{true},
                vk::PhysicalDeviceDynamicRenderingFeatures{true}
            };

            device = physicalDevice.createDevice(createInfoChain.get<vk::DeviceCreateInfo>());
            VULKAN_HPP_DEFAULT_DISPATCHER.init(device);

            queue = device.getQueue(queueFamily, 0);
        }

        // Create command pool
        {
            vk::CommandPoolCreateInfo poolInfo;
            poolInfo.setFlags(vk::CommandPoolCreateFlagBits::eResetCommandBuffer);
            poolInfo.setQueueFamilyIndex(queueFamily);
            commandPool = device.createCommandPool(poolInfo);
        }

        // Create descriptor pool
        {
            std::vector<vk::DescriptorPoolSize> poolSizes{
                {vk::DescriptorType::eUniformBuffer, 1},
                {vk::DescriptorType::eAccelerationStructureKHR, 1}
            };

            vk::DescriptorPoolCreateInfo poolInfo;
            poolInfo.setPoolSizes(poolSizes);
            poolInfo.setMaxSets(3);
            poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
            descriptorPool = device.createDescriptorPool(poolInfo);
        }
    }

    static uint32_t findMemoryType(uint32_t typeFilter, vk::MemoryPropertyFlags properties)
    {
        vk::PhysicalDeviceMemoryProperties memProperties = physicalDevice.getMemoryProperties();
        for (uint32_t i = 0; i != memProperties.memoryTypeCount; ++i) {
            if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
                return i;
            }
        }
        throw std::runtime_error("failed to find suitable memory type");
    }

    static vk::UniqueDescriptorSet allocateDescSet(vk::DescriptorSetLayout descSetLayout)
    {
        return std::move(device.allocateDescriptorSetsUnique({ descriptorPool, descSetLayout }).front());
    }

    static std::vector<vk::UniqueCommandBuffer> allocateCommandBuffers(size_t count)
    {
        vk::CommandBufferAllocateInfo allocInfo;
        allocInfo.setCommandPool(commandPool);
        allocInfo.setCommandBufferCount(static_cast<uint32_t>(count));
        return device.allocateCommandBuffersUnique(allocInfo);
    }

    static void oneTimeSubmit(const std::function<void(vk::CommandBuffer)>& func)
    {
        vk::UniqueCommandBuffer cmdBuf = std::move(allocateCommandBuffers(1).front());
        cmdBuf->begin({ vk::CommandBufferUsageFlagBits::eOneTimeSubmit });
        func(*cmdBuf);
        cmdBuf->end();

        vk::SubmitInfo submitInfo;
        submitInfo.setCommandBuffers(*cmdBuf);
        queue.submit(submitInfo);
        queue.waitIdle();
    }

    static void terminate()
    {
        device.destroyDescriptorPool(descriptorPool);
        device.destroyCommandPool(commandPool);
        device.destroy();
        instance.destroySurfaceKHR(surface);
        instance.destroyDebugUtilsMessengerEXT(messenger);
        instance.destroy();
    }

    static inline vk::Instance instance;
    static inline vk::DebugUtilsMessengerEXT messenger;
    static inline vk::SurfaceKHR surface;
    static inline vk::Device device;
    static inline vk::PhysicalDevice physicalDevice;
    static inline uint32_t queueFamily;
    static inline vk::Queue queue;
    static inline vk::CommandPool commandPool;
    static inline vk::DescriptorPool descriptorPool;
};
