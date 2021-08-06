#define VULKAN_HPP_DISPATCH_LOADER_DYNAMIC 1
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE

#include <set>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <fstream>
#include <optional>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <vulkan/vulkan.hpp>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

VULKAN_HPP_DEFAULT_DISPATCH_LOADER_DYNAMIC_STORAGE

using vkIL = vk::ImageLayout;
using vkA = vk::AccessFlagBits;
using vkBU = vk::BufferUsageFlagBits;
using vkMP = vk::MemoryPropertyFlagBits;
using vkDT = vk::DescriptorType;

const bool enableValidationLayers = true;
const int MAX_FRAMES_IN_FLIGHT = 2;

const std::vector<const char*> validationLayers = {
    "VK_LAYER_KHRONOS_validation"
};

const std::vector<const char*> deviceExtensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME,
    VK_KHR_MAINTENANCE3_EXTENSION_NAME,
    VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
    VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME,
    VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
    VK_KHR_RAY_QUERY_EXTENSION_NAME,
};

struct QueueFamilyIndices
{
    std::optional<uint32_t> graphicsFamily;
    std::optional<uint32_t> presentFamily;

    bool isComplete()
    {
        return graphicsFamily.has_value() && presentFamily.has_value();
    }
};

struct SwapChainSupportDetails
{
    vk::SurfaceCapabilitiesKHR capabilities;
    std::vector<vk::SurfaceFormatKHR> formats;
    std::vector<vk::PresentModeKHR> presentModes;
};

struct Vertex
{
    glm::vec3 pos;
    glm::vec3 normal;
    uint16_t meshIndex;

    static vk::VertexInputBindingDescription getBindingDescription()
    {
        vk::VertexInputBindingDescription bindingDescription;
        bindingDescription.setBinding(0);
        bindingDescription.setStride(sizeof(Vertex));
        bindingDescription.setInputRate(vk::VertexInputRate::eVertex);
        return bindingDescription;
    }

    static std::array<vk::VertexInputAttributeDescription, 3> getAttributeDescriptions()
    {
        std::array<vk::VertexInputAttributeDescription, 3> attributeDescriptions;
        attributeDescriptions[0].setBinding(0);
        attributeDescriptions[0].setLocation(0);
        attributeDescriptions[0].setFormat(vk::Format::eR32G32B32Sfloat);
        attributeDescriptions[0].setOffset(offsetof(Vertex, pos));

        attributeDescriptions[1].setBinding(0);
        attributeDescriptions[1].setLocation(1);
        attributeDescriptions[1].setFormat(vk::Format::eR32G32B32Sfloat);
        attributeDescriptions[1].setOffset(offsetof(Vertex, normal));

        attributeDescriptions[2].setBinding(0);
        attributeDescriptions[2].setLocation(2);
        attributeDescriptions[2].setFormat(vk::Format::eR16Uint);
        attributeDescriptions[2].setOffset(offsetof(Vertex, meshIndex));
        return attributeDescriptions;
    }
};

uint32_t findMemoryType(vk::PhysicalDevice physicalDevice, uint32_t typeFilter,
                        vk::MemoryPropertyFlags properties)
{
    vk::PhysicalDeviceMemoryProperties memProperties = physicalDevice.getMemoryProperties();
    for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
        if ((typeFilter & (1 << i)) &&
            (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
            return i;
        }
    }
    throw std::runtime_error("failed to find suitable memory type!");
}

struct Buffer
{
    vk::Device device;
    vk::UniqueBuffer buffer;
    vk::UniqueDeviceMemory memory;
    vk::DeviceSize size;
    vk::BufferUsageFlags usage;
    uint64_t deviceAddress;
    vk::DescriptorBufferInfo bufferInfo;
    void* mapped = nullptr;

    void create(vk::Device device, vk::DeviceSize size, vk::BufferUsageFlags usage)
    {
        this->device = device;
        this->size = size;
        this->usage = usage;
        buffer = device.createBufferUnique({ {}, size, usage });
    }

    void allocate(vk::PhysicalDevice physicalDevice, vk::MemoryPropertyFlags properties)
    {
        vk::MemoryRequirements requirements = device.getBufferMemoryRequirements(*buffer);
        uint32_t type = findMemoryType(physicalDevice, requirements.memoryTypeBits, properties);

        vk::MemoryAllocateInfo allocInfo{ requirements.size, type };
        if (usage & vk::BufferUsageFlagBits::eShaderDeviceAddress) {
            vk::MemoryAllocateFlagsInfo flagsInfo{ vk::MemoryAllocateFlagBits::eDeviceAddress };
            allocInfo.pNext = &flagsInfo;

            memory = device.allocateMemoryUnique(allocInfo);
            device.bindBufferMemory(*buffer, *memory, 0);

            vk::BufferDeviceAddressInfoKHR bufferDeviceAI{ *buffer };
            deviceAddress = device.getBufferAddressKHR(&bufferDeviceAI);
        } else {
            memory = device.allocateMemoryUnique(allocInfo);
            device.bindBufferMemory(*buffer, *memory, 0);
        }
    }

    void copy(void* data)
    {
        if (!mapped) {
            mapped = device.mapMemory(*memory, 0, size);
        }
        memcpy(mapped, data, static_cast<size_t>(size));
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


struct Image
{
    vk::Device device;
    vk::UniqueImage image;
    vk::UniqueImageView view;
    vk::UniqueDeviceMemory memory;
    vk::Extent2D extent;
    vk::Format format;
    vk::ImageLayout imageLayout;
    vk::ImageAspectFlags aspect;

    void create(vk::Device device, vk::Extent2D extent,
                vk::Format format, vk::ImageUsageFlags usage)
    {
        this->device = device;
        this->extent = extent;
        this->format = format;

        vk::ImageCreateInfo createInfo;
        createInfo.setImageType(vk::ImageType::e2D);
        createInfo.setExtent({ extent.width, extent.height, 1 });
        createInfo.setMipLevels(1);
        createInfo.setArrayLayers(1);
        createInfo.setFormat(format);
        createInfo.setTiling(vk::ImageTiling::eOptimal);
        createInfo.setUsage(usage);
        image = device.createImageUnique(createInfo);
        imageLayout = vkIL::eUndefined;
    }

    void allocate(vk::PhysicalDevice physicalDevice)
    {
        vk::MemoryRequirements requirements = device.getImageMemoryRequirements(*image);
        uint32_t memoryTypeIndex = findMemoryType(physicalDevice, requirements.memoryTypeBits,
                                                  vk::MemoryPropertyFlagBits::eDeviceLocal);
        memory = device.allocateMemoryUnique({ requirements.size, memoryTypeIndex });
        device.bindImageMemory(*image, *memory, 0);
    }

    void createView(vk::ImageAspectFlags aspect)
    {
        this->aspect = aspect;
        vk::ImageViewCreateInfo createInfo;
        createInfo.setImage(*image);
        createInfo.setViewType(vk::ImageViewType::e2D);
        createInfo.setFormat(format);
        createInfo.setSubresourceRange({ aspect, 0, 1, 0, 1 });
        view = device.createImageViewUnique(createInfo);
    }

    bool hasStencilComponent(vk::Format format)
    {
        return format == vk::Format::eD32SfloatS8Uint || format == vk::Format::eD24UnormS8Uint;
    }

    void transitionImageLayout(vk::CommandBuffer commandBuffer, vk::ImageLayout newLayout)
    {
        vk::ImageMemoryBarrier barrier;
        barrier.setOldLayout(imageLayout);
        barrier.setNewLayout(newLayout);
        barrier.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barrier.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barrier.setImage(*image);
        barrier.setSubresourceRange({ aspect, 0, 1, 0, 1 });

        vk::PipelineStageFlags srcStage;
        vk::PipelineStageFlags dstStage;

        if (imageLayout == vkIL::eUndefined && newLayout == vkIL::eTransferDstOptimal) {
            barrier.srcAccessMask = {};
            barrier.dstAccessMask = vkA::eTransferWrite;
            srcStage = vk::PipelineStageFlagBits::eTopOfPipe;
            dstStage = vk::PipelineStageFlagBits::eTransfer;
        } else if (imageLayout == vkIL::eTransferDstOptimal &&
                   newLayout == vkIL::eShaderReadOnlyOptimal) {
            barrier.srcAccessMask = vkA::eTransferWrite;
            barrier.dstAccessMask = vkA::eShaderRead;
            srcStage = vk::PipelineStageFlagBits::eTransfer;
            dstStage = vk::PipelineStageFlagBits::eFragmentShader;
        } else if (imageLayout == vkIL::eUndefined &&
                   newLayout == vkIL::eDepthStencilAttachmentOptimal) {
            barrier.srcAccessMask = {};
            barrier.dstAccessMask = (vkA::eDepthStencilAttachmentRead |
                                     vkA::eDepthStencilAttachmentWrite);
            srcStage = vk::PipelineStageFlagBits::eTopOfPipe;
            dstStage = vk::PipelineStageFlagBits::eEarlyFragmentTests;
        } else {
            throw std::invalid_argument("unsupported layout transition!");
        }

        if (newLayout == vk::ImageLayout::eDepthStencilAttachmentOptimal) {
            barrier.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eDepth;

            if (hasStencilComponent(format)) {
                barrier.subresourceRange.aspectMask |= vk::ImageAspectFlagBits::eStencil;
            }
        } else {
            barrier.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        }

        commandBuffer.pipelineBarrier(srcStage, dstStage, {}, {}, {}, barrier);
        imageLayout = newLayout;
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

        uint16_t currentMeshIndex = 0;
        while (std::getline(file, line)) {
            std::vector<std::string> list = split(line, ' ');
            if (list[0] == "v") {
                positions.push_back(glm::vec3(stof(list[1]), -stof(list[2]), stof(list[3])));
            }
            if (list[0] == "vn") {
                normals.push_back(glm::vec3(stof(list[1]), -stof(list[2]), stof(list[3])));
            }
            if (list[0] == "o") {
                if (list[1] == "Teapot") currentMeshIndex = 0;
                if (list[1] == "Floor") currentMeshIndex = 1;
                if (list[1] == "Bunny") currentMeshIndex = 2;
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

    vk::Device device;
    vk::PhysicalDevice physicalDevice;
    vk::AccelerationStructureTypeKHR type;
    uint32_t primitiveCount;
    vk::DeviceSize size;
    vk::AccelerationStructureBuildGeometryInfoKHR geometryInfo;
    uint64_t deviceAddress;
    vk::WriteDescriptorSetAccelerationStructureKHR asInfo;

    void createBuffer(vk::Device device,
                      vk::PhysicalDevice physicalDevice,
                      vk::AccelerationStructureGeometryKHR geometry,
                      vk::AccelerationStructureTypeKHR type,
                      uint32_t primitiveCount)
    {
        this->device = device;
        this->physicalDevice = physicalDevice;
        this->type = type;
        this->primitiveCount = primitiveCount;

        geometryInfo.setType(type);
        geometryInfo.setFlags(vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
        geometryInfo.setGeometries(geometry);

        vk::AccelerationStructureBuildSizesInfoKHR buildSizesInfo =
            device.getAccelerationStructureBuildSizesKHR(vk::AccelerationStructureBuildTypeKHR::eDevice, geometryInfo, primitiveCount);
        size = buildSizesInfo.accelerationStructureSize;
        buffer.create(device, size, vkBU::eAccelerationStructureStorageKHR | vkBU::eShaderDeviceAddress);
        buffer.allocate(physicalDevice, vkMP::eDeviceLocal);
    }

    void create()
    {
        vk::AccelerationStructureCreateInfoKHR createInfo;
        createInfo.setBuffer(*buffer.buffer);
        createInfo.setSize(size);
        createInfo.setType(type);
        handle = device.createAccelerationStructureKHRUnique(createInfo);
    }

    void build(vk::CommandBuffer commandBuffer)
    {
        Buffer scratchBuffer;
        scratchBuffer.create(device, size, vkBU::eStorageBuffer | vkBU::eShaderDeviceAddress);
        scratchBuffer.allocate(physicalDevice, vkMP::eDeviceLocal);
        geometryInfo.setScratchData(scratchBuffer.deviceAddress);
        geometryInfo.setDstAccelerationStructure(*handle);

        vk::AccelerationStructureBuildRangeInfoKHR rangeInfo{ primitiveCount , 0, 0, 0 };
        commandBuffer.buildAccelerationStructuresKHR(geometryInfo, &rangeInfo);
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
    void initWindow(uint32_t width, uint32_t height)
    {
        glfwInit();
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        window = glfwCreateWindow(width, height, "Vulkan", nullptr, nullptr);
    }

    void initVulkan()
    {
        createInstance();
        setupDebugMessenger();
        createSurface();
        pickPhysicalDevice();
        createLogicalDevice();
        createSwapChain();
        createImageViews();
        createRenderPass();
        createDescriptorSetLayout();
        createGraphicsPipeline();
        createCommandPool();
        createDepthResources();
        createFramebuffers();
        createVertexBuffer();
        createIndexBuffer();
        createUniformBuffers();
        createBottomLevelAS();
        createTopLevelAS();
        createDescriptorPool();
        createDescriptorSets();
        createCommandBuffers();
        createSyncObjects();
    }

    void mainLoop()
    {
        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();
            drawFrame();
        }
        device->waitIdle();
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
    GLFWwindow* window;

    vk::UniqueInstance instance;
    vk::UniqueDebugUtilsMessengerEXT debugMessenger;
    vk::UniqueSurfaceKHR surface;

    vk::PhysicalDevice physicalDevice;
    vk::UniqueDevice device;

    vk::Queue graphicsQueue;
    vk::Queue presentQueue;

    vk::UniqueSwapchainKHR swapChain;
    std::vector<vk::Image> swapChainImages;
    vk::Format swapChainImageFormat;
    vk::Extent2D swapChainExtent;

    std::vector<vk::UniqueImageView> swapChainImageViews;
    std::vector<vk::UniqueFramebuffer> swapChainFramebuffers;
    Image depthImage;

    vk::UniqueDescriptorSetLayout descriptorSetLayout;
    vk::UniqueDescriptorPool descriptorPool;
    std::vector<vk::UniqueDescriptorSet> descriptorSets;

    std::vector<Vertex> vertices;
    std::vector<uint16_t> indices;
    std::string vertShaderPath;
    std::string fragShaderPath;
    vk::UniqueRenderPass renderPass;
    vk::UniquePipelineLayout pipelineLayout;
    vk::UniquePipeline graphicsPipeline;

    Buffer vertexBuffer;
    Buffer indexBuffer;
    std::vector<Buffer> uniformBuffers;
    AccelerationStructure bottomLevelAS;
    AccelerationStructure topLevelAS;

    vk::UniqueCommandPool commandPool;
    std::vector<vk::UniqueCommandBuffer> commandBuffers;

    std::vector<vk::UniqueSemaphore> imageAvailableSemaphores;
    std::vector<vk::UniqueSemaphore> renderFinishedSemaphores;
    std::vector<vk::Fence> inFlightFences;
    std::vector<vk::Fence> imagesInFlight;
    size_t currentFrame = 0;


    void cleanup()
    {
        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            device->destroyFence(inFlightFences[i]);
        }
        glfwDestroyWindow(window);
        glfwTerminate();
    }

    void createInstance()
    {
        static vk::DynamicLoader dl;
        auto vkGetInstanceProcAddr = dl.getProcAddress<PFN_vkGetInstanceProcAddr>("vkGetInstanceProcAddr");
        VULKAN_HPP_DEFAULT_DISPATCHER.init(vkGetInstanceProcAddr);

        if (enableValidationLayers && !checkValidationLayerSupport()) {
            throw std::runtime_error("validation layers requested, but not available!");
        }

        vk::ApplicationInfo appInfo;
        appInfo.setApiVersion(VK_API_VERSION_1_2);

        auto extensions = getRequiredExtensions();
        vk::InstanceCreateInfo createInfo{ {}, &appInfo, {}, extensions };
        if (enableValidationLayers) {
            createInfo.setPEnabledLayerNames(validationLayers);
        }
        instance = vk::createInstanceUnique(createInfo, nullptr);

        VULKAN_HPP_DEFAULT_DISPATCHER.init(*instance);
    }

    void setupDebugMessenger()
    {
        if (!enableValidationLayers) return;
        using vkMS = vk::DebugUtilsMessageSeverityFlagBitsEXT;
        using vkMT = vk::DebugUtilsMessageTypeFlagBitsEXT;
        vk::DebugUtilsMessengerCreateInfoEXT createInfo;
        createInfo.setMessageSeverity(vkMS::eWarning | vkMS::eError);
        createInfo.setMessageType(vkMT::eGeneral | vkMT::ePerformance | vkMT::eValidation);
        createInfo.setPfnUserCallback(&debugUtilsMessengerCallback);
        debugMessenger = instance->createDebugUtilsMessengerEXTUnique(createInfo);
    }

    void createSurface()
    {
        VkSurfaceKHR _surface;
        VkInstance _instance = *instance;
        if (glfwCreateWindowSurface(_instance, window, nullptr, &_surface) != VK_SUCCESS) {
            throw std::runtime_error("failed to create window surface!");
        }
        surface = vk::UniqueSurfaceKHR(vk::SurfaceKHR(_surface), { *instance });
    }

    void pickPhysicalDevice()
    {
        std::vector<vk::PhysicalDevice> devices = instance->enumeratePhysicalDevices();
        for (const auto& device : devices) {
            if (isDeviceSuitable(device)) {
                physicalDevice = device;
                break;
            }
        }
        if (!physicalDevice) {
            throw std::runtime_error("failed to find a suitable GPU!");
        }
    }

    void createLogicalDevice()
    {
        QueueFamilyIndices indices = findQueueFamilies(physicalDevice);
        uint32_t graphicsFamily = indices.graphicsFamily.value();
        uint32_t presentFamily = indices.presentFamily.value();
        std::set<uint32_t> uniqueQueueFamilies{ graphicsFamily, presentFamily };

        std::vector<vk::DeviceQueueCreateInfo> queueCreateInfos;
        float queuePriority = 1.0f;
        for (uint32_t queueFamily : uniqueQueueFamilies) {
            vk::DeviceQueueCreateInfo queueCreateInfo{ {}, queueFamily, 1, &queuePriority };
            queueCreateInfos.push_back(queueCreateInfo);
        }

        vk::PhysicalDeviceFeatures deviceFeatures;
        vk::DeviceCreateInfo createInfo;
        createInfo.setQueueCreateInfos(queueCreateInfos);
        createInfo.setPEnabledExtensionNames(deviceExtensions);
        createInfo.setPEnabledFeatures(&deviceFeatures);
        if (enableValidationLayers) {
            createInfo.setPEnabledLayerNames(validationLayers);
        }

        vk::StructureChain createInfoChain{ createInfo,
            vk::PhysicalDeviceBufferDeviceAddressFeatures{true},
            vk::PhysicalDeviceAccelerationStructureFeaturesKHR{true},
            vk::PhysicalDeviceRayQueryFeaturesKHR{true} };

        device = physicalDevice.createDeviceUnique(createInfoChain.get<vk::DeviceCreateInfo>());
        graphicsQueue = device->getQueue(graphicsFamily, 0);
        presentQueue = device->getQueue(presentFamily, 0);
    }

    void createSwapChain()
    {
        SwapChainSupportDetails swapChainSupport = querySwapChainSupport(physicalDevice);

        vk::SurfaceFormatKHR surfaceFormat = chooseSwapSurfaceFormat(swapChainSupport.formats);
        vk::PresentModeKHR presentMode = chooseSwapPresentMode(swapChainSupport.presentModes);
        vk::Extent2D extent = chooseSwapExtent(swapChainSupport.capabilities);

        uint32_t imageCount = swapChainSupport.capabilities.minImageCount + 1;
        if (swapChainSupport.capabilities.maxImageCount > 0 &&
            imageCount > swapChainSupport.capabilities.maxImageCount) {
            imageCount = swapChainSupport.capabilities.maxImageCount;
        }

        vk::SwapchainCreateInfoKHR createInfo;
        createInfo.setSurface(*surface);
        createInfo.setMinImageCount(imageCount);
        createInfo.setImageFormat(surfaceFormat.format);
        createInfo.setImageColorSpace(surfaceFormat.colorSpace);
        createInfo.setImageExtent(extent);
        createInfo.setImageArrayLayers(1);
        createInfo.setImageUsage(vk::ImageUsageFlagBits::eColorAttachment);
        createInfo.setImageSharingMode(vk::SharingMode::eExclusive);
        createInfo.setPreTransform(swapChainSupport.capabilities.currentTransform);
        createInfo.setCompositeAlpha(vk::CompositeAlphaFlagBitsKHR::eOpaque);
        createInfo.setPresentMode(presentMode);
        createInfo.setClipped(VK_TRUE);

        QueueFamilyIndices indices = findQueueFamilies(physicalDevice);
        if (indices.graphicsFamily != indices.presentFamily) {
            uint32_t graphicsFamily = indices.graphicsFamily.value();
            uint32_t presentFamily = indices.presentFamily.value();
            std::array familyIndices{ graphicsFamily, presentFamily };
            createInfo.setImageSharingMode(vk::SharingMode::eConcurrent);
            createInfo.setQueueFamilyIndices(familyIndices);
        }

        swapChain = device->createSwapchainKHRUnique(createInfo);
        swapChainImages = device->getSwapchainImagesKHR(*swapChain);
        swapChainImageFormat = surfaceFormat.format;
        swapChainExtent = extent;
    }

    void createImageViews()
    {
        swapChainImageViews.resize(swapChainImages.size());

        vk::ImageSubresourceRange subresourceRange{ vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1 };
        for (size_t i = 0; i < swapChainImages.size(); i++) {
            vk::ImageViewCreateInfo createInfo;
            createInfo.setImage(swapChainImages[i]);
            createInfo.setViewType(vk::ImageViewType::e2D);
            createInfo.setFormat(swapChainImageFormat);
            createInfo.setSubresourceRange(subresourceRange);
            swapChainImageViews[i] = device->createImageViewUnique(createInfo);
        }
    }

    void createRenderPass()
    {
        vk::AttachmentDescription colorAttachment;
        colorAttachment.setFormat(swapChainImageFormat);
        colorAttachment.setLoadOp(vk::AttachmentLoadOp::eClear);
        colorAttachment.setStoreOp(vk::AttachmentStoreOp::eStore);
        colorAttachment.setStencilLoadOp(vk::AttachmentLoadOp::eDontCare);
        colorAttachment.setStencilStoreOp(vk::AttachmentStoreOp::eDontCare);
        colorAttachment.setFinalLayout(vk::ImageLayout::ePresentSrcKHR);

        vk::AttachmentDescription depthAttachment;
        depthAttachment.setFormat(findDepthFormat());
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
        vk::SubpassDependency dependency;
        dependency.setSrcSubpass(VK_SUBPASS_EXTERNAL);
        dependency.setDstSubpass(0);
        dependency.setSrcStageMask(vkPS::eColorAttachmentOutput | vkPS::eEarlyFragmentTests);
        dependency.setDstStageMask(vkPS::eColorAttachmentOutput | vkPS::eEarlyFragmentTests);
        dependency.setDstAccessMask(vkA::eColorAttachmentWrite | vkA::eDepthStencilAttachmentWrite);

        std::array attachments{ colorAttachment, depthAttachment };
        vk::RenderPassCreateInfo renderPassInfo({}, attachments, subpass, dependency);
        renderPass = device->createRenderPassUnique(renderPassInfo);
    }

    vk::UniqueShaderModule createShaderModule(const std::string& shaderPath)
    {
        auto code = readFile(shaderPath);
        vk::ShaderModuleCreateInfo createInfo;
        createInfo.setCodeSize(code.size());
        createInfo.setPCode(reinterpret_cast<const uint32_t*>(code.data()));
        return device->createShaderModuleUnique(createInfo);
    }

    void createDescriptorSetLayout()
    {
        using vkSS = vk::ShaderStageFlagBits;
        std::vector<vk::DescriptorSetLayoutBinding> bindings;
        bindings.push_back({ 0, vkDT::eUniformBuffer, 1, vkSS::eVertex });
        bindings.push_back({ 1, vkDT::eAccelerationStructureKHR, 1, vkSS::eFragment });

        vk::DescriptorSetLayoutCreateInfo layoutInfo;
        layoutInfo.setBindings(bindings);
        descriptorSetLayout = device->createDescriptorSetLayoutUnique(layoutInfo);
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

        float width = static_cast<float>(swapChainExtent.width);
        float height = static_cast<float>(swapChainExtent.height);
        vk::Viewport viewport{ 0.0f, 0.0f, width, height, 0.0f, 1.0f };
        vk::Rect2D scissor{ { 0, 0 }, swapChainExtent };
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

        std::array dynamicStates{ vk::DynamicState::eViewport, vk::DynamicState::eLineWidth };
        vk::PipelineDynamicStateCreateInfo dynamicState({}, dynamicStates);

        vk::PipelineLayoutCreateInfo pipelineLayoutInfo;
        pipelineLayoutInfo.setSetLayouts(*descriptorSetLayout);
        pipelineLayout = device->createPipelineLayoutUnique(pipelineLayoutInfo);

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
        pipelineInfo.setRenderPass(*renderPass);
        pipelineInfo.setSubpass(0);

        auto result = device->createGraphicsPipelineUnique({}, pipelineInfo);
        if (result.result != vk::Result::eSuccess) {
            throw std::runtime_error("failed to create a pipeline!");
        }
        graphicsPipeline = std::move(result.value);
    }

    void createFramebuffers()
    {
        swapChainFramebuffers.reserve(swapChainImageViews.size());
        for (auto const& view : swapChainImageViews) {
            std::array attachments{ *view, *depthImage.view };
            vk::FramebufferCreateInfo framebufferInfo;
            framebufferInfo.setRenderPass(*renderPass);
            framebufferInfo.setAttachments(attachments);
            framebufferInfo.setWidth(swapChainExtent.width);
            framebufferInfo.setHeight(swapChainExtent.height);
            framebufferInfo.setLayers(1);
            swapChainFramebuffers.push_back(device->createFramebufferUnique(framebufferInfo));
        }
    }

    void createCommandPool()
    {
        QueueFamilyIndices queueFamilyIndices = findQueueFamilies(physicalDevice);
        vk::CommandPoolCreateInfo poolInfo({}, queueFamilyIndices.graphicsFamily.value());
        commandPool = device->createCommandPoolUnique(poolInfo);
    }

    vk::Format findSupportedFormat(const std::vector<vk::Format>& candidates,
                                   vk::ImageTiling tiling, vk::FormatFeatureFlags features)
    {
        for (vk::Format format : candidates) {
            vk::FormatProperties props = physicalDevice.getFormatProperties(format);
            if (tiling == vk::ImageTiling::eLinear &&
                (props.linearTilingFeatures & features) == features) {
                return format;
            } else if (tiling == vk::ImageTiling::eOptimal &&
                       (props.optimalTilingFeatures & features) == features) {
                return format;
            }
        }
        throw std::runtime_error("failed to find supported format!");
    }

    vk::Format findDepthFormat()
    {
        return findSupportedFormat(
            { vk::Format::eD32Sfloat, vk::Format::eD32SfloatS8Uint, vk::Format::eD24UnormS8Uint },
            vk::ImageTiling::eOptimal,
            vk::FormatFeatureFlagBits::eDepthStencilAttachment);
    }

    void createDepthResources()
    {
        vk::Format depthFormat = findDepthFormat();
        vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eDepthStencilAttachment;
        depthImage.create(*device, swapChainExtent, depthFormat, usage);
        depthImage.allocate(physicalDevice);
        depthImage.createView(vk::ImageAspectFlagBits::eDepth);

        vk::UniqueCommandBuffer commandBuffer = beginSingleTimeCommands();
        vk::ImageLayout layout = vk::ImageLayout::eDepthStencilAttachmentOptimal;
        depthImage.transitionImageLayout(*commandBuffer, layout);
        endSingleTimeCommands(*commandBuffer);
    }

    void createVertexBuffer()
    {
        vk::DeviceSize size = sizeof(Vertex) * vertices.size();
        vk::BufferUsageFlags usage{
            vkBU::eAccelerationStructureBuildInputReadOnlyKHR |
            vkBU::eStorageBuffer |
            vkBU::eShaderDeviceAddress |
            vkBU::eVertexBuffer };
        vertexBuffer.create(*device, size, usage);
        vertexBuffer.allocate(physicalDevice, vkMP::eHostVisible | vkMP::eHostCoherent);
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
        indexBuffer.create(*device, size, usage);
        indexBuffer.allocate(physicalDevice, vkMP::eHostVisible | vkMP::eHostCoherent);
        indexBuffer.copy(indices.data());
    }

    void createUniformBuffers()
    {
        vk::DeviceSize size = sizeof(UniformBufferObject);
        uniformBuffers.resize(swapChainImages.size());
        for (size_t i = 0; i < swapChainImages.size(); i++) {
            uniformBuffers[i].create(*device, size, vk::BufferUsageFlagBits::eUniformBuffer);
            uniformBuffers[i].allocate(physicalDevice, vkMP::eHostVisible | vkMP::eHostCoherent);
        }
    }

    void updateUniformBuffer(uint32_t currentImage)
    {
        using namespace std::chrono;
        static auto startTime = high_resolution_clock::now();
        auto currentTime = high_resolution_clock::now();
        float time = duration<float, seconds::period>(currentTime - startTime).count();

        float angle = time * glm::radians(60.0f);
        glm::mat4 rotate = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, -1.0f, 0.0f));
        glm::vec3 camPos = glm::vec3(rotate * glm::vec4(12.0f, -12.0f, 12.0f, 1.0f));
        float aspect = swapChainExtent.width / (float)swapChainExtent.height;

        UniformBufferObject ubo;
        ubo.model = glm::mat4(1.0f);
        ubo.view = glm::lookAt(camPos, glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f));
        ubo.proj = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);
        ubo.proj[1][1] *= -1;

        uniformBuffers[currentImage].copy(&ubo);
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
        bottomLevelAS.createBuffer(*device, physicalDevice, geometry,
                                   vk::AccelerationStructureTypeKHR::eBottomLevel, primitiveCount);
        bottomLevelAS.create();
        vk::UniqueCommandBuffer cmdBuf = beginSingleTimeCommands();
        bottomLevelAS.build(*cmdBuf);
        endSingleTimeCommands(*cmdBuf);
    }

    void createTopLevelAS()
    {
        VkTransformMatrixKHR transformMatrix = { 1.0f, 0.0f, 0.0f, 0.0f,
                                                 0.0f, 1.0f, 0.0f, 0.0f,
                                                 0.0f, 0.0f, 1.0f, 0.0f };
        vk::AccelerationStructureInstanceKHR asInstance;
        asInstance.setTransform(transformMatrix);
        asInstance.setMask(0xFF);
        asInstance.setAccelerationStructureReference(bottomLevelAS.buffer.deviceAddress);
        asInstance.setFlags(vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable);

        Buffer instancesBuffer;
        instancesBuffer.create(*device, sizeof(vk::AccelerationStructureInstanceKHR),
                               vkBU::eAccelerationStructureBuildInputReadOnlyKHR |
                               vkBU::eShaderDeviceAddress);
        instancesBuffer.allocate(physicalDevice, vkMP::eHostVisible | vkMP::eHostCoherent);
        instancesBuffer.copy(&asInstance);

        vk::AccelerationStructureGeometryInstancesDataKHR instancesData;
        instancesData.setArrayOfPointers(false);
        instancesData.setData(instancesBuffer.deviceAddress);

        vk::AccelerationStructureGeometryKHR geometry;
        geometry.setGeometryType(vk::GeometryTypeKHR::eInstances);
        geometry.setGeometry({ instancesData });
        geometry.setFlags(vk::GeometryFlagBitsKHR::eOpaque);

        uint32_t primitiveCount = 1;
        topLevelAS.createBuffer(*device, physicalDevice, geometry,
                                vk::AccelerationStructureTypeKHR::eTopLevel, primitiveCount);
        topLevelAS.create();
        vk::UniqueCommandBuffer cmdBuf = beginSingleTimeCommands();
        topLevelAS.build(*cmdBuf);
        endSingleTimeCommands(*cmdBuf);
    }

    void createDescriptorPool()
    {
        std::vector<vk::DescriptorPoolSize> poolSizes{
            { vkDT::eUniformBuffer, 1 },
            { vkDT::eAccelerationStructureKHR, 1 } };

        vk::DescriptorPoolCreateInfo poolInfo;
        poolInfo.setPoolSizes(poolSizes);
        poolInfo.setMaxSets(static_cast<uint32_t>(swapChainImages.size()));
        poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
        descriptorPool = device->createDescriptorPoolUnique(poolInfo);
    }

    void createDescriptorSets()
    {
        std::vector layouts{ swapChainImages.size(), *descriptorSetLayout };
        vk::DescriptorSetAllocateInfo allocInfo;
        allocInfo.setDescriptorPool(*descriptorPool);
        allocInfo.setSetLayouts(layouts);

        descriptorSets.resize(swapChainImages.size());
        descriptorSets = device->allocateDescriptorSetsUnique(allocInfo);

        for (size_t i = 0; i < swapChainImages.size(); i++) {
            std::vector<vk::WriteDescriptorSet> descriptorWrites{
                uniformBuffers[i].createDescWrite(*descriptorSets[i], vkDT::eUniformBuffer, 0),
                topLevelAS.createDescWrite(*descriptorSets[i], 1) };
            device->updateDescriptorSets(descriptorWrites, nullptr);
        }
    }

    void createCommandBuffers()
    {
        vk::CommandBufferAllocateInfo allocInfo;
        allocInfo.setCommandPool(*commandPool);
        allocInfo.setCommandBufferCount(static_cast<uint32_t>(swapChainFramebuffers.size()));
        commandBuffers = device->allocateCommandBuffersUnique(allocInfo);

        std::array<vk::ClearValue, 2> clearValues;
        clearValues[0].color = { std::array{0.0f, 0.0f, 0.0f, 1.0f} };
        clearValues[1].depthStencil = vk::ClearDepthStencilValue{ 1.0f, 0 };

        vk::RenderPassBeginInfo renderPassInfo;
        renderPassInfo.setRenderPass(*renderPass);
        renderPassInfo.setRenderArea({ {0, 0}, swapChainExtent });
        renderPassInfo.setClearValues(clearValues);
        for (size_t i = 0; i < commandBuffers.size(); i++) {
            renderPassInfo.setFramebuffer(*swapChainFramebuffers[i]);
            commandBuffers[i]->begin(vk::CommandBufferBeginInfo{});
            commandBuffers[i]->beginRenderPass(renderPassInfo, vk::SubpassContents::eInline);
            commandBuffers[i]->bindPipeline(vk::PipelineBindPoint::eGraphics, *graphicsPipeline);

            vk::DeviceSize offsets{ 0 };
            commandBuffers[i]->bindVertexBuffers(0, *vertexBuffer.buffer, offsets);
            commandBuffers[i]->bindIndexBuffer(*indexBuffer.buffer, 0, vk::IndexType::eUint16);
            commandBuffers[i]->bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                                                  *pipelineLayout, 0, *descriptorSets[i], nullptr);
            commandBuffers[i]->drawIndexed(static_cast<uint32_t>(indices.size()), 1, 0, 0, 0);

            commandBuffers[i]->endRenderPass();
            commandBuffers[i]->end();
        }
    }

    vk::UniqueCommandBuffer beginSingleTimeCommands()
    {
        vk::CommandBufferAllocateInfo allocInfo;
        allocInfo.setCommandPool(*commandPool);
        allocInfo.setCommandBufferCount(1);
        auto commandBuffers = device->allocateCommandBuffersUnique(allocInfo);
        vk::UniqueCommandBuffer commandBuffer = std::move(commandBuffers.front());
        commandBuffer->begin({ vk::CommandBufferUsageFlagBits::eOneTimeSubmit });
        return commandBuffer;
    }

    void endSingleTimeCommands(vk::CommandBuffer commandBuffer)
    {
        commandBuffer.end();

        vk::UniqueFence fence = device->createFenceUnique({});
        vk::SubmitInfo submitInfo;
        submitInfo.setCommandBuffers(commandBuffer);
        graphicsQueue.submit(submitInfo, *fence);

        vk::Result res = device->waitForFences(*fence, true, UINT64_MAX);
        if (res != vk::Result::eSuccess) {
            throw std::runtime_error("failed to wait for fences");
        }
    }

    void createSyncObjects()
    {
        imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
        renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
        inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);
        imagesInFlight.resize(swapChainImages.size());

        for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
            imageAvailableSemaphores[i] = device->createSemaphoreUnique({});
            renderFinishedSemaphores[i] = device->createSemaphoreUnique({});
            inFlightFences[i] = device->createFence({ vk::FenceCreateFlagBits::eSignaled });
        }
    }

    uint32_t acquireNextImage(size_t currentFrame)
    {
        vk::Semaphore semaphore = *imageAvailableSemaphores[currentFrame];
        auto result = device->acquireNextImageKHR(*swapChain, UINT64_MAX, semaphore);
        if (result.result != vk::Result::eSuccess) {
            throw std::runtime_error("failed to acquire next image!");
        }
        return result.value;
    }

    void drawFrame()
    {
        device->waitForFences(inFlightFences[currentFrame], true, UINT64_MAX);
        device->resetFences(inFlightFences[currentFrame]);

        uint32_t imageIndex = acquireNextImage(currentFrame);
        if (imagesInFlight[imageIndex]) {
            device->waitForFences(imagesInFlight[imageIndex], true, UINT64_MAX);
        }
        imagesInFlight[imageIndex] = inFlightFences[currentFrame];
        device->resetFences(inFlightFences[currentFrame]);

        updateUniformBuffer(imageIndex);

        vk::PipelineStageFlags waitStage{ vk::PipelineStageFlagBits::eColorAttachmentOutput };
        vk::Semaphore imageAvailableSemaphore = *imageAvailableSemaphores[currentFrame];
        vk::Semaphore renderFinishedSemaphore = *renderFinishedSemaphores[currentFrame];
        vk::SubmitInfo submitInfo;
        submitInfo.setWaitSemaphores(imageAvailableSemaphore);
        submitInfo.setWaitDstStageMask(waitStage);
        submitInfo.setCommandBuffers(*commandBuffers[imageIndex]);
        submitInfo.setSignalSemaphores(renderFinishedSemaphore);
        graphicsQueue.submit(submitInfo, inFlightFences[currentFrame]);

        vk::PresentInfoKHR presentInfo{ renderFinishedSemaphore, *swapChain, imageIndex };
        graphicsQueue.presentKHR(presentInfo);

        currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
    }

    vk::SurfaceFormatKHR chooseSwapSurfaceFormat(
        const std::vector<vk::SurfaceFormatKHR>& availableFormats)
    {
        for (const auto& availableFormat : availableFormats) {
            if (availableFormat.format == vk::Format::eB8G8R8A8Srgb &&
                availableFormat.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear) {
                return availableFormat;
            }
        }
        return availableFormats[0];
    }

    vk::PresentModeKHR chooseSwapPresentMode(
        const std::vector<vk::PresentModeKHR>& availablePresentModes)
    {
        for (const auto& availablePresentMode : availablePresentModes) {
            if (availablePresentMode == vk::PresentModeKHR::eFifoRelaxed) {
                return availablePresentMode;
            }
        }
        return vk::PresentModeKHR::eFifo;
    }

    vk::Extent2D chooseSwapExtent(const vk::SurfaceCapabilitiesKHR& capabilities)
    {
        if (capabilities.currentExtent.width != UINT32_MAX) {
            return capabilities.currentExtent;
        } else {
            int width, height;
            glfwGetFramebufferSize(window, &width, &height);
            vk::Extent2D actualExtent{ static_cast<uint32_t>(width), static_cast<uint32_t>(height) };
            vk::Extent2D minExtent = capabilities.minImageExtent;
            vk::Extent2D maxExtent = capabilities.maxImageExtent;
            actualExtent.width = std::clamp(actualExtent.width, minExtent.width, maxExtent.width);
            actualExtent.height = std::clamp(actualExtent.height, minExtent.height, maxExtent.height);
            return actualExtent;
        }
    }

    SwapChainSupportDetails querySwapChainSupport(vk::PhysicalDevice device)
    {
        SwapChainSupportDetails details;
        details.capabilities = device.getSurfaceCapabilitiesKHR(*surface);
        details.formats = device.getSurfaceFormatsKHR(*surface);
        details.presentModes = device.getSurfacePresentModesKHR(*surface);
        return details;
    }

    bool isDeviceSuitable(vk::PhysicalDevice device)
    {
        QueueFamilyIndices indices = findQueueFamilies(device);
        bool extensionsSupported = checkDeviceExtensionSupport(device);
        bool swapChainAdequate = false;
        if (extensionsSupported) {
            SwapChainSupportDetails swapChainSupport = querySwapChainSupport(device);
            bool formatEmpty = swapChainSupport.formats.empty();
            bool presentModeEmpty = swapChainSupport.presentModes.empty();
            swapChainAdequate = !formatEmpty && !presentModeEmpty;
        }
        return indices.isComplete() && extensionsSupported && swapChainAdequate;
    }

    bool checkDeviceExtensionSupport(vk::PhysicalDevice device)
    {
        auto availableExtensions = device.enumerateDeviceExtensionProperties();
        std::set<std::string> requiredExtensions(deviceExtensions.begin(), deviceExtensions.end());
        for (const auto& extension : availableExtensions) {
            requiredExtensions.erase(extension.extensionName);
        }
        return requiredExtensions.empty();
    }

    QueueFamilyIndices findQueueFamilies(vk::PhysicalDevice device)
    {
        QueueFamilyIndices indices;
        std::vector<vk::QueueFamilyProperties> queueFamilies = device.getQueueFamilyProperties();
        uint32_t i = 0;
        for (const auto& queueFamily : queueFamilies) {
            if (queueFamily.queueFlags & vk::QueueFlagBits::eGraphics) {
                indices.graphicsFamily = i;
            }
            VkBool32 presentSupport = device.getSurfaceSupportKHR(i, *surface);
            if (presentSupport) {
                indices.presentFamily = i;
            }
            if (indices.isComplete()) break;
            i++;
        }
        return indices;
    }

    std::vector<const char*> getRequiredExtensions()
    {
        uint32_t glfwExtensionCount = 0;
        const char** glfwExtensions;
        glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
        std::vector<const char*> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);
        if (enableValidationLayers) {
            extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        }
        return extensions;
    }

    bool checkValidationLayerSupport()
    {
        std::vector<vk::LayerProperties> availableLayers = vk::enumerateInstanceLayerProperties();
        for (const char* layerName : validationLayers) {
            bool layerFound = false;
            for (const auto& layerProperties : availableLayers) {
                if (strcmp(layerName, layerProperties.layerName) == 0) {
                    layerFound = true;
                    break;
                }
            }
            if (!layerFound) return false;
        }
        return true;
    }

    static std::vector<char> readFile(const std::string& filename)
    {
        std::ifstream file(filename, std::ios::ate | std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("failed to open file!");
        }
        size_t fileSize = (size_t)file.tellg();
        std::vector<char> buffer(fileSize);
        file.seekg(0);
        file.read(buffer.data(), fileSize);
        file.close();
        return buffer;
    }

    static VKAPI_ATTR VkBool32 VKAPI_CALL debugUtilsMessengerCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
        VkDebugUtilsMessageTypeFlagsEXT messageTypes,
        VkDebugUtilsMessengerCallbackDataEXT const* pCallbackData,
        void* pUserData)
    {
        std::cerr << pCallbackData->pMessage << std::endl << std::endl;
        return VK_FALSE;
    }
};

int main()
{
    Mesh mesh;
    mesh.load("assets/bunny_and_teapot.obj");

    Application app;
    app.setVertShaderPath("shaders/spv/shader.vert.spv");
    app.setFragShaderPath("shaders/spv/shader.frag.spv");
    app.setMesh(mesh);

    try {
        app.initWindow(1000, 800);
        app.initVulkan();
        app.mainLoop();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
